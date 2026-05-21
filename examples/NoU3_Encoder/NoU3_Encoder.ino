/**
 * Example code for an encodered motor controlled with PestoLink: https://pestol.ink
 * The NoU3 documentation and tutorials can be found at https://alfredo-nou3.readthedocs.io/
 */

#include <PestoLink-Receive.h>
#include <Alfredo_NoU3.h>

//motor ports 2-7 by default define their pins themselves!
NoU_Motor defaultEncoderedMotor(2);

//motor ports 1 and 8 can be assigned custom gpio for an encoder, like the servo pins (but why do this when you could use the jst ports?)
NoU_Motor customEncoderedMotor(1);

//you can also create an encoder unassociated with a motor. Keep in mind your robot can only use up to 8 total encoders
Encoder customEncoder;

void setup() {
    NoU3.begin();

    //if using port 2-7, all you have to do is call beginEncoder()
    defaultEncoderedMotor.beginEncoder();

    //you can create an encoder for M1 and M8, but you have to supply your own pins
    int enc2_pinA = 4;
    int enc2_pinB = 5;
    customEncoderedMotor.beginEncoder(enc2_pinA, enc2_pinB);

    //you have to supply your own pins for unassociated encoders too
    int enc3_pinA = 6;
    int enc3_pinB = 7;
    customEncoder.begin(enc3_pinA, enc3_pinB);
    
    
    PestoLink.begin("NoU3_Encodered_Motor");
    Serial.begin(115200);

}

unsigned long lastPrintTime = 0;

void loop() {
    //print encoder positions 10 times a second
    if (lastPrintTime + 100 < millis()){
        Serial.print(defaultEncoderedMotor.getPosition());
        Serial.print(" ");
        Serial.print(customEncoderedMotor.getPosition());
        Serial.print(" ");
        Serial.print(customEncoder.getPosition());
        Serial.println(" ");
        lastPrintTime = millis();
    }
    
    //Connect with PestoLink to drive the motors 
    if (PestoLink.isConnected()) {
        float throttle = PestoLink.getAxis(1);
        
        defaultEncoderedMotor.set(throttle);
        customEncoderedMotor.set(throttle);

        NoU3.setServiceLight(LIGHT_ENABLED);
    } else {
        NoU3.stopMotors();
        NoU3.setServiceLight(LIGHT_DISABLED);
    }
}
