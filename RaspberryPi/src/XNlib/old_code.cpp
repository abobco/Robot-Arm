  // ideal ik test code
  // arm.print();
  // arm.resolve(target);
  // printf("\nafter resolution\n\n");
  // arm.print();

  // float end_bone_lengths[] = {
  //   vec3::dist(arm.bones[0].position, arm.bones[1].position),
  //   vec3::dist(arm.bones[1].position, arm.bones[2].position)
  // };

// servp angle print
cout << "servo angles\n";
for ( int i =0; i < phys_arm.ideal_chain.bone_count; i++ ) {
  int j = 0;
  for ( pio::ServoAngular& s : phys_arm.servos[i] ) {
    printf("servo_ang[%d, %d] = %f\n", i, j++, s.getAngle() );
  }
} 

// test constant bone lengths
for ( int i = 0; i < 2; i++ ){
  float d = start_bone_lengths[i] - vec3::dist(phys_arm.positions[i], phys_arm.positions[i+1]);
  cout << start_bone_lengths[i] << " " << vec3::dist(phys_arm.positions[i], phys_arm.positions[i+1]) << " " << d << '\n';
  assert(abs(d) < SMEPSILON);
}

// draw a box rotated by servo angles
void draw_bone(vec3& position, const Shader &shader, ik::IkConstraint& constraint, vector<pio::ServoAngular>& servos, int isWireFrame=false) {
  if ( isWireFrame ){
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
  } else {
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  }

  glm::vec3 scale(1,1,1); 
  glm::vec3 pos(position.x, position.y, position.z );
  glm::mat4 model = glm::mat4(1.0f); 
  model = glm::translate(model, pos);
  // model = glm::scale(model, scale);
  for ( int i = 0; i < constraint.num_axes; i++ ) {
    glm::vec3 ax(constraint.axes[i].x, constraint.axes[i].y, constraint.axes[i].z );
    constraint.axes[i].print();
    DUMP(servos[i].getAngle());
    model = glm::rotate(model, servos[i].getAngle(), ax);
  }  
    shader.setMat4("model", model);
    glDrawArrays(GL_TRIANGLES, 0, 36);
}

 // lws servo control 
 if ( get_connections() ) { 
      // manual servo control
      vec3 d = get_motion_data(0);
      servo_cam_x.setPosition(d.x / (M_PI*0.5f));           
    } else if (target_x != 0 && false) {
        if ( target_x < cam_mid.x - cam_dead_zone_x ) {
            servo_cam_x.setPosition(servo_cam_x.getPosition() + step);
            servo_arm_x.setPosition(servo_arm_x.getPosition() + step);
        } else if ( target_x > cam_mid.x + cam_dead_zone_x ) {
            servo_cam_x.setPosition(servo_cam_x.getPosition() - step);
            servo_arm_x.setPosition(servo_arm_x.getPosition() - step);
        } else if ( target_y < cam_mid.y - cam_dead_zone_y ) {
            servo_cam_y.setPosition(servo_cam_y.getPosition() + step); 
            servo_arm_y.setPosition(servo_arm_y.getPosition() - step); 
        } else if ( target_y > cam_mid.y + cam_dead_zone_y ) {
            servo_cam_y.setPosition(servo_cam_y.getPosition() - step);
            servo_arm_y.setPosition(servo_arm_y.getPosition() + step);
        }
        // else 
        //     servo_cam_x.setPosition(0);
      // }
      double sleepy_joe = (1.0 - abs(target_x - cam_mid.x)/cam_mid.x)*0.2;
      time_sleep(sleepy_joe);
    }

    xn::pio::TimePoint servo_sweep_search(xn::pio::TimePoint t_start, xn::pio::ServoContinuous& servo) {
    float step_dur = 0.25;
    int step_width = 25;

    static int dir = 1;
    auto t_now =  xn::pio::get_time();
    double sweep_dur = xn::pio::time_diff_seconds(t_start, t_now);
    // printf("sweep_dur=%f\n", sweep_dur);

    if ( sweep_dur > 3 ) {
      t_start = t_now;
      dir *= -1;
    }
    
    if ( fmod(sweep_dur, step_dur) > step_dur/4)
      servo.setPosition(0);
    else {
      int w = step_width;
      if ( dir < 0 ) w++;
      servo.setWidthFromCenter(w*dir);
    }
      // printf("%d\n", w*dir);
    return t_start;
}