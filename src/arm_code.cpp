// #include "main.h"

// //pros::Motor arm(-18, pros::MotorGears::red);
// pros::Rotation armRotation(15);

// const int numStates = 2;
// int states[numStates] = {30, 200};
// int currState = 0;
// int target = 0;
// int multiplier = 350;
// bool armPIDEnabled = true;

// void nextState(){
//   currState = (currState + 1) % numStates;
//   target = states[currState];
// }

// void liftControl(){
//   double kp = 2;
//   double error = target - armRotation.get_position();
//   double velocity = kp * error;
//   arm.move(velocity);
// }

// void initialize(){
//   pros::Task liftControlTask([]{
//     while(true){
//       if(armPIDEnabled) {
//         liftControl();
//       }
//       pros::delay(10);
//     }
//   });
// }

// void opcontrol(){
//   pros::Controller master(pros::E_CONTROLLER_MASTER);
//   while(true){
//     if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1) || master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)){
//       armPIDEnabled = false;
//     }
//     if(armRotation.get_position() <= 5 && master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)){
//        multiplier = 0;
//     }
//     else if(armRotation.get_position() >= 300 && master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)){
//       multiplier = 0;
//     }
//     else{
//       mulitplier = 350;
//     }
//     if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)){
//       armPIDEnabled = true;
//       nextState();
//     }
//     pros::delay(20);
//   }
// }
