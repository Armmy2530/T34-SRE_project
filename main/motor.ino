void fd(int speed){
    M1.setSpeed(speed);
    M2.setSpeed(speed);
}
void st(){
    M1.setSpeed(0);
    M2.setSpeed(0);
}
void tr(int speed){
    M1.setSpeed(speed);
    M2.setSpeed(0);
}
void tl(int speed){
    M1.setSpeed(0);
    M2.setSpeed(speed);
}
void sr(int speed){
    M1.setSpeed(speed);
    M2.setSpeed(-speed);
}
void sl(int speed){
    M1.setSpeed(-speed);
    M2.setSpeed(speed);
}