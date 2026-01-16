//Note: This implementation is a sequential execution model for proper placement accuracy.
//This code represents the action for 2 robots- collaborating to build a 3x3 grid block- with high accuracy and precision
// R1 - Fills 2 columns, R2 - Fills 1 column
#include <Servo.h>

bool hasRun = false;
//black robot - R1
Servo bleft;
Servo bright;
Servo bbase;
Servo bgrp;

//wooden robot - R2
Servo wleft;
Servo wright;
Servo wbase;
Servo wgrp;

void setup() {
 bgrp.attach(2);
 bgrp.write(80);
delay(1000);
bleft.attach(3);
bright.attach(4);
bbase.attach(5);
bbase.write(110);
delay(1000); // Attach servo3 to pin 3*/

wgrp.attach(8);
wgrp.write(70);
delay(1000);
wleft.attach(10);
wright.attach(9);
wright.write(40);
delay(1000);
wbase.attach(11);
wbase.write(85);
delay(1000);
}

// block for smoothly moving 
void fron(Servo &servo, int st, int en) {
for (int pos = st; pos <= en; pos=pos+1) {//0 to 90
servo.write(pos);
delay(50);}
delay(1000);
}

void bak(Servo &servo, int st, int en) {
for (int pos = st; pos >= en; pos--) {//90 to 0
servo.write(pos);
delay(50);}
delay(1000);
}

void blacktak(){
bleft.write(90);
delay(100);
bright.write(90);
delay(100);
delay(1000);
bak(bbase,110,20);
bgrp.write(60);
delay(2000);
//go front
fron(bright,90,120);
delay(2000);
bgrp.write(85);
delay(100);

bak(bright,120,90);
delay(1000);

fron(bbase,20,110);
}

void blacklev(){
bgrp.write(74);
delay(1000);
bright.write(90);
delay(1000);
bleft.write(90);
delay(1000);
bbase.write(100);
delay(100);
delay(1000);
}
void woodtak(){
wleft.write(90);
delay(100);
wright.write(40);// 40 is right 
delay(100);
delay(1000);

fron(wbase,85,163);

wgrp.write(60);
delay(2000);

wleft.write(130);
delay(100);

fron(wright,40,70);
delay(2000);

wgrp.write(95);
delay(2000);

wright.write(40);
delay(100);

bak(wbase,163,85);

}

void woodlev(){
wgrp.write(55);
delay(1000);
wright.write(40);//40 is normal position
delay(500);
wleft.write(90);
delay(500);

wbase.write(85);
delay(100);
delay(1000);
  
}

//31
void black31(){
fron(bbase,110,125);
bleft.write(90);
delay(100);
fron(bright,90,160);
delay(3000);
}

//32
void black32(){
fron(bbase,110,110);
bleft.write(95);
delay(100);
fron(bright,90,155);
delay(3000);
}


//22
void black22(){
fron(bbase,110,110);
bleft.write(103);
delay(100);
fron(bright,90,149);
delay(3000);
}

//21
void black21(){
fron(bbase,110,125);
bleft.write(110);
delay(100);
fron(bright,90,153);
delay(3000);
}


//11
void black11(){
fron(bbase,110,123);
bleft.write(133);
delay(100);
fron(bright,90,140);
delay(3000);
}
//12
void black12(){
fron(bbase,110,110);
bleft.write(130);
delay(100);
fron(bright,90,134);
delay(3000);
}
//33
void wood33(){
fron(wbase,85,85);
wleft.write(108);
delay(500);
fron(wright,40,97);
delay(3000);
}
//23
void wood23(){

fron(wbase,85,85);
wleft.write(115);
delay(500);
fron(wright,40,75);
delay(3000);
}
//13
void wood13(){

fron(wbase,85,85);
wlwrite(136);
delayeft.(500);
fron(wright,40,57);
delay(3000);
}


void loop() {

if (!hasRun) {
// 31
blacktak();
black31();
blacklev();
delay(3000);
// 33
woodtak();
wood33();
woodlev();
delay(3000);
//32
blacktak();
black32();
blacklev();
delay(3000);
//23
woodtak();
wood23();
woodlev();
delay(3000);
//22
blacktak();
black22();
blacklev();
delay(3000);
// 13
woodtak();
wood13();
woodlev();
delay(3000);
//21
blacktak();
black21();
blacklev();
delay(3000);


//11
blacktak();
black11();
blacklev();
delay(3000);
//12
blacktak();
black12();
blacklev();
delay(3000);


 hasRun = true; 
}}




