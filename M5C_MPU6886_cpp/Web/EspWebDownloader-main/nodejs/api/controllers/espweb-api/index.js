'use strict';

const HELPER_BASE = process.env.HELPER_BASE || "/opt/";
const Response = require(HELPER_BASE + 'response');
const Redirect = require(HELPER_BASE + 'redirect');

const PUBLIC_FOLDER = "";
const PUBLIC_FIRMWARE_FOLDER = process.env.THIS_BASE_PATH + "/public" + PUBLIC_FOLDER + "/firmwares/";
const PUBLIC_TEMPLATE_FOLDER = process.env.THIS_BASE_PATH + "/public" + PUBLIC_FOLDER + "/template/";

const fs = require('fs').promises;
const crypto = require('crypto');

const BOOTLOADER_FNAME = "bootloader.bin";
const BOOTAPP_FNAME = "boot_app0.bin";
const PARTITIONS_FNAME = "partitions.bin";
const FIRMWARE_FNAME = "firmware.bin";
const MANIFEST_FNAME = "manifest.json";
const APP_FNAME = "app.json";

const chipFamlies = [{
	chipFamily: "ESP32",
	bootloader_offset: 0x1000,
},
{
	chipFamily: "ESP32-C3",
	bootloader_offset: 0x0000,
},
{
	chipFamily: "ESP32-S3",
	bootloader_offset: 0x0000,
}
];

exports.handler = async (event, context, callback) => {
	var body = JSON.parse(event.body);
	console.log(body);

	if( event.path == '/espweb-list-app'){
		var dir = await fs.readdir(PUBLIC_FIRMWARE_FOLDER);
		console.log(dir);
		var list = [];
		for( let item of dir ){
			var buffer = await fs.readFile(PUBLIC_FIRMWARE_FOLDER + item + "/" + APP_FNAME);
			var json = JSON.parse(Buffer.from(buffer));
			list.push(json);
		}
		return new Response({ list });
	}else

	if( event.path == '/espweb-add-app'){
		var name = body.name;
		var memo = body.memo || "";

		var uuid = crypto.randomUUID();
		var dir = PUBLIC_FIRMWARE_FOLDER + uuid;

		var now = new Date().getTime();
		await fs.mkdir(dir);
		var json = {
			name: name,
			uuid: uuid,
			memo: memo,
			created_at: now,
			model_list: []
		};
		await fs.writeFile(dir + "/" + APP_FNAME, JSON.stringify(json, null, "\t"));
		return new Response({ uuid: uuid });
	}else

	if( event.path == '/espweb-update-app'){
		var uuid = body.uuid;
		var name = body.name;
		var memo = body.memo || "";
		if( !check_uuid(uuid) )
			throw new Error("invalid uuid");

		var buffer = await fs.readFile(PUBLIC_FIRMWARE_FOLDER + uuid + "/" + APP_FNAME);
		var json = JSON.parse(Buffer.from(buffer));
		json.name = name;
		json.memo = memo;
		await fs.writeFile(PUBLIC_FIRMWARE_FOLDER + uuid + "/" + APP_FNAME, JSON.stringify(json, null, "\t"));

		return new Response({ uuid: uuid });
	}else

	if( event.path == '/espweb-remove-app'){
		var app = body.app;
		if( !check_uuid(app) )
			throw new Error("invalid uuid");

		await fs.rm(PUBLIC_FIRMWARE_FOLDER + app , { recursive: true });
		
		return new Response({});
	}else

	if( event.path == '/espweb-add-model'){
		var app = body.app;
		var name = body.name;
		var chip_family = body.chip_family;
		var memo = body.memo || "";

		if( !check_uuid(app) )
			throw new Error("invalid app");

		if( (!event.files.file_bootloader || event.files.file_bootloader.length <= 0 ) || 
			(!event.files.file_partitions || event.files.file_partitions.length <= 0 ) || 
			(!event.files.file_firmware || event.files.file_firmware.length <= 0 ) )
			throw new Error("file not uploaded");

    var chipFamily = chipFamlies.find(item => item.chipFamily == chip_family);
		if( !chipFamily )
			throw new Error("invalid chip family");

		var now = new Date().getTime();
		var manifest = {
				name: name,
				builds: [
						{
								chipFamily: chipFamily.chipFamily,
								improv: false,
								parts: [
										{
												path: BOOTLOADER_FNAME,
												offset: chipFamily.bootloader_offset
										},
										{
												path: PARTITIONS_FNAME,
												offset: 0x8000
										},
										{
												path: BOOTAPP_FNAME,
												offset: 0xe000
										},
										{
												path: FIRMWARE_FNAME,
												offset: 0x10000
										}
								]
						},
				]
		};

		var uuid = crypto.randomUUID();
		var dir = PUBLIC_FIRMWARE_FOLDER + app + "/" + uuid;
		await fs.mkdir(dir);
		await fs.writeFile(dir + "/" + MANIFEST_FNAME, JSON.stringify(manifest, null, "\t"));
		await fs.writeFile(dir + "/" + BOOTLOADER_FNAME, event.files.file_bootloader[0].buffer);
		await fs.writeFile(dir + "/" + PARTITIONS_FNAME, event.files.file_partitions[0].buffer);
		await fs.writeFile(dir + "/" + FIRMWARE_FNAME, event.files.file_firmware[0].buffer);
		await fs.copyFile(PUBLIC_TEMPLATE_FOLDER + BOOTAPP_FNAME, dir + "/" + BOOTAPP_FNAME);

		var buffer = await fs.readFile(PUBLIC_FIRMWARE_FOLDER + app + "/" + APP_FNAME);
		var json = JSON.parse(Buffer.from(buffer));
		json.model_list.push({
			name: manifest.name,
			uuid: uuid,
			chip_family: chipFamily.chipFamily,
			created_at: now,
			memo: memo,
		});
		await fs.writeFile(PUBLIC_FIRMWARE_FOLDER + app + "/" + APP_FNAME, JSON.stringify(json, null, "\t"));

		return new Response({ uuid: uuid });
	}else

	if( event.path == '/espweb-update-model' ){
		var app = body.app;
		var uuid = body.uuid;
		var name = body.name;
		var memo = body.memo || "";

		if( !check_uuid(app) || !check_uuid(uuid) )
			throw new Error("invalid uuid");

		var buffer = await fs.readFile(PUBLIC_FIRMWARE_FOLDER + app + "/" + uuid + "/" + MANIFEST_FNAME);
		var json = JSON.parse(Buffer.from(buffer));
		json.name = name;
		await fs.writeFile(PUBLIC_FIRMWARE_FOLDER + app + "/" + uuid + "/" + MANIFEST_FNAME, JSON.stringify(json, null, "\t"));

		var buffer = await fs.readFile(PUBLIC_FIRMWARE_FOLDER + app + "/" + APP_FNAME);
		var json = JSON.parse(Buffer.from(buffer));
		var item = json.model_list.find(item => item.uuid == uuid);
		if( item ){
			item.name = name;
			item.memo = memo;
		}
		await fs.writeFile(PUBLIC_FIRMWARE_FOLDER + app + "/" + APP_FNAME, JSON.stringify(json, null, "\t"));

		return new Response({ app: app, uuid: uuid });
	}else

	if( event.path == '/espweb-remove-model'){
		var app = body.app;
		var uuid = body.uuid;
		if( !check_uuid(app) || !check_uuid(uuid) )
			throw new Error("invalid uuid");

		var dir = await fs.readdir(PUBLIC_FIRMWARE_FOLDER + app);
		var item = dir.find(item => item == uuid);
		if( !item )
			throw new Error('uuid not found');

		await fs.rm(PUBLIC_FIRMWARE_FOLDER + app + "/" + item, { recursive: true });
		
		var buffer = await fs.readFile(PUBLIC_FIRMWARE_FOLDER + app + "/" + APP_FNAME);
		var json = JSON.parse(Buffer.from(buffer));
		var index = json.model_list.findIndex(item => item.uuid == uuid);
		if( index >= 0 )
			json.model_list.splice(index, 1);
		await fs.writeFile(PUBLIC_FIRMWARE_FOLDER + app + "/" + APP_FNAME, JSON.stringify(json, null, "\t"));
		
		return new Response({});
	}else

	{
		throw new Error('unknown endpoint');
	}
};

function check_uuid(uuid){
//	const regex = /[a-zA-Z0-9/-]/g;
	const regex = /([0-9a-fA-F]{8})-([0-9a-fA-F]{4})-([0-9a-fA-F]{4})-([0-9a-fA-F]{4})-([0-9a-fA-F]{12})/g;
	return regex.test(uuid);
}
