/*
#define RIGHT_LITTLE 1
#define RIGHT_RING 2
#define RIGHT_MIDDLE 3 
#define RIGHT_INDEX 4
*/
//#edfin RIGHT_THUMB 9

class IllumiTrack(){
    private:
    
    public:
        void setup(){
            pinMode(RIGHT_LITTLE, INPUT);
            pinMode(RIGHT_RING, INPUT);
            pinMode(RIGHT_MIDDLE, INPUT);
            pinMode(RIGHT_INDEX, INPUT);
            //pinMode(RIGHT_THUMB, INPUT);
        };
        void loop();
};