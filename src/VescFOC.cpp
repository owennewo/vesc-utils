#include <VescFOC.h>
#include <SimpleFOC.h>

VescFOC::VescFOC(int _pole_pairs) {
    // sensor
    pole_pairs = _pole_pairs;
  motor = new BLDCMotor(_pole_pairs);  //_motor;
  driver = new BLDCDriver6PWM(H1,L1,H2,L2,H3,L3,EN_GATE);  // _driver;
  sensor = new HallSensor(HALL_1, HALL_2, HALL_3, _pole_pairs); //_sensor;
}

void VescFOC::begin() {
  
  driver->pwm_frequency = NOT_SET;
  
  attachInterrupt(digitalPinToInterrupt(sensor->pinA), std::bind(&HallSensor::handleA, sensor), CHANGE);
  attachInterrupt(digitalPinToInterrupt(sensor->pinB), std::bind(&HallSensor::handleB, sensor), CHANGE);
  attachInterrupt(digitalPinToInterrupt(sensor->pinC), std::bind(&HallSensor::handleC, sensor), CHANGE);
  sensor->init();
  motor->linkSensor(sensor);

  driver->voltage_power_supply = 24;
  driver->init();
  motor->linkDriver(driver);
  
  motor->voltage_sensor_align = 0.75;
  motor->voltage_limit = 1.5;

  motor->PID_velocity.P = 0.35;
  motor->PID_velocity.I = 2.0;
  // motor->LPF_velocity.Tf = 0.001;
  

  motor->controller = MotionControlType::velocity;
  motor->torque_controller = TorqueControlType::voltage;

  motor->useMonitoring(Serial);

  motor->init();
  motor->initFOC();

  motor->shaft_angle = 0.0; // for angle_openloop (ovf)
  
}

void VescFOC::move(float target) {
  motor->move(target);
}

void VescFOC::loop() {
  // Serial.println(target);
    
    motor->loopFOC();
}

void VescFOC::print(Stream& printer) {
    Serial.println(motor->shaft_angle);
}
