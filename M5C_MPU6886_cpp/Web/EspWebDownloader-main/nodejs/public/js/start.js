'use strict';

//const vConsole = new VConsole();
//const remoteConsole = new RemoteConsole("http://[remote server]/logio-post");
//window.datgui = new dat.GUI();

const base_url = "";

var vue_options = {
    el: "#top",
    mixins: [mixins_bootstrap],
    store: vue_store,
    router: vue_router,
    data: {
        target_app: {},
        params_add_app: {},
        params_add_model: {},
        params_update_model: {},
        params_update_app: {},
        firmware_list: [],
    },
    computed: {
    },
    methods: {
        start_update_model: function(index, index2){
            this.target_app = this.firmware_list[index];
            var params = JSON.parse(JSON.stringify(this.target_app.model_list[index2]));
            params.app = this.target_app.uuid;
            this.params_update_model = params;
            this.dialog_open('#update_model_dialog');
        },
        do_update_model: async function(){
            if( !confirm('更新しますか？') )
                return;

            try{
                this.params_update_model.app = this.target_app.uuid;
                var result = await do_post(base_url + "/espweb-update-model", this.params_update_model);
                console.log(result);
                
                await this.list_update();
                this.dialog_close('#update_model_dialog');
            }catch(error){
                console.error(error);
                alert(error);
            }
        },

        start_update_app: function(index){
            this.target_app = this.firmware_list[index];
            this.params_update_app = JSON.parse(JSON.stringify(this.target_app));
            this.dialog_open('#update_app_dialog');
        },
        do_update_app: async function(){
            if( !confirm('更新しますか？') )
                return;

            try{
                var result = await do_post(base_url + "/espweb-update-app", this.params_update_app);
                console.log(result);

                await this.list_update();
                this.dialog_close('#update_app_dialog');
            }catch(error){
                console.error(error);
                alert(error);
            }
        },

        start_add_app: function(){
            this.params_add_app = {};
            this.dialog_open('#add_app_dialog');
        },
        do_add_app: async function(){
            try{
                var result = await do_post(base_url + "/espweb-add-app", this.params_add_app);
                console.log(result);

                await this.list_update();
                this.dialog_close('#add_app_dialog');
            }catch(error){
                console.error(error);
                alert(error);
            }
        },

        start_add_model: function(index){
            this.target_app = this.firmware_list[index];
            this.params_add_model = {
                app: this.target_app.uuid
            };
            this.dialog_open('#add_model_dialog');
        },
        do_add_model: async function(){
            try{
                var result = await do_post_formdata(base_url + "/espweb-add-model", this.params_add_model);
                console.log(result);

                await this.list_update();
                this.dialog_close('#add_model_dialog');
            }catch(error){
                console.error(error);
                alert(error);
            }
        },

        do_remove_model: async function(index, index2){
            if( !confirm('本当に削除しますか？'))
                return;

            try{
                var params = {
                    app: this.firmware_list[index].uuid,
                    uuid: this.firmware_list[index].model_list[index2].uuid,
                };
                var result = await do_post(base_url + "/espweb-remove-model", params);
                console.log(result);

                await this.list_update();
                this.dialog_close('#remove_model_dialog');
            }catch(error){
                console.error(error);
                alert(error);
            }
        },

        do_remove_app: async function(index){
            if( !confirm('本当に削除しますか？') )
                return;

            try{
                var params = {
                    app: this.firmware_list[index].uuid,
                };
                var result = await do_post(base_url + "/espweb-remove-app", params);
                console.log(result);

                await this.list_update();
                this.dialog_close('#info_app_dialog');
            }catch(error){
                console.error(error);
                alert(error);
            }
        },

        bootloader_change: function(files){
            if( files.length <= 0 ){
                this.params_add_model.file_bootloader = null;
                return;
            }
            this.params_add_model.file_bootloader = files[0];
        },
        partitions_change: function(files){
            if( files.length <= 0 ){
                this.params_add_model.file_partitions = null;
                return;
            }
            this.params_add_model.file_partitions = files[0];
        },
        firmware_change: function(files){
            if( files.length <= 0 ){
                this.params_add_model.file_firmware = null;
                return;
            }
            this.params_add_model.file_firmware = files[0];
        },

        list_update: async function(){
            try{
                var list = await do_post(base_url + "/espweb-list-app");
                console.log(list);
                this.firmware_list = list.list;
            }catch(error){
                console.error(error);
                alert(error);
            }
        },
    },
    created: function(){
    },
    mounted: async function(){
        proc_load();

        await this.list_update();
    }
};
vue_add_data(vue_options, { progress_title: '' }); // for progress-dialog
vue_add_global_components(components_bootstrap);
vue_add_global_components(components_utils);

/* add additional components */
  
window.vue = new Vue( vue_options );
