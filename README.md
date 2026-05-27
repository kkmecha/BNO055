# BNO055
個人的に使いやすいようにこっちに上げただけです  
  
main.cpp  
BNO055 imu(PB_9, PB_8);  
float imu_ang[3] = {0.f};  
  
void yaw_filter_easy() {
    float qw = imu.quat.w, qx = imu.quat.x, qy = imu.quat.y, qz = imu.quat.z;
    float t3 = +2.0f * (qw * qz + qx * qy);
    float t4 = +1.0f - 2.0f * (qy * qy + qz * qz);  
    imu_ang[0] = std::atan2(t3, t4); 
}  

void odometry() { 
    __disable_irq();
    now_angles[0] = encoder_raw[0];
    now_angles[1] = encoder_raw[1];
    __enable_irq();
  
    float d_wheel_x = (now_angles[0] - prev_angles[0]) * M_PI * 0.0508f / 360.0f;
    float d_wheel_y = (now_angles[1] - prev_angles[1]) * M_PI * 0.0508f / 360.0f; 
    float yaw = imu_ang[0]; // R(-Θ)として計算するため

    float delta_yaw = yaw - prev_yaw;
    d_wheel_x -= offset_y * delta_yaw;
    d_wheel_y += offset_x * delta_yaw;

    _pos_x += d_wheel_x * cos(yaw) - d_wheel_y * sin(yaw);
    _pos_y += d_wheel_x * sin(yaw) + d_wheel_y * cos(yaw); 
    prev_angles[0] = now_angles[0]; prev_angles[1] = now_angles[1];
    prev_yaw = yaw;
}  
  

int main(){
    while(!imu.check()){ leds[1] = !leds[1]; ThisThread::sleep_for(100ms); }
    imu.reset(); ThisThread::sleep_for(500ms);
    imu.setmode(OPERATION_MODE_NDOF); ThisThread::sleep_for(100ms);
    imu.SetExternalCrystal(true);

    while(true) {
        imu.get_calib();
        if(imu.calib > 0) break; 
        leds[1] = !leds[1]; ThisThread::sleep_for(100ms);
    }
    imu.get_quat_async();
    
    while(true){
        if (!imu_i2c_busy) { 
            imu.update_quat_data();
            yaw_filter_easy(); 
            imu.get_quat_async(); 
        }
    }
