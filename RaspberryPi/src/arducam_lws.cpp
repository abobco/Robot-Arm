#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <signal.h>
#include <assert.h> 
#include <string.h>
#include <pthread.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h> 
#include <math.h>

extern "C" {
#include "ws/ws_ring_server.c"
}

#include "XNlib/xn_gpio.hpp"
#include "XNlib/xn_ik.hpp"

#define laptop_port 8080 
#define laptop_ip "192.168.0.183"

bool start_read_data = false;
bool receiving_commands = true;
bool move_to_target = false;
bool face_found = false;

int sock = 0;

xn::ik::ServoChain *armptr;
xn::pio::SmoothServo *clawptr, *wristptr;
xn::pio::ServoAngular servo_cam_x(23, 500, 2500);
xn::pio::ServoAngular servo_cam_y(24, 500, 2500);

const float w_to_ang = M_PI / (2500 - 500); 
const float claw_open_a = (1100 - 500 )*w_to_ang;
const float claw_close_a = (1500 - 500)*w_to_ang;

const xn::vec2 servo_cam_origin{1675, 853};
const int servo_cam_y_perp_w = 1050;

xn::vec3 target = {0,0,0};
xn::vec2 cam_ang_diff = {0, 0};
xn::vec2 cam_ang_orig = {0, 0};
double wristlen = 150;

pthread_mutex_t frameread_lock; 

void *cam_io_thread(void *arg);
void *servo_ctl_thread(void *arg);
void  INThandler(int sig);
void read_int32_t(int sock, int32_t *intptr );
void *cam_search_thread(void *argv);

int main(int argc, char *argv[])
{
  pthread_t cam_io_tid, servo_ctl_tid, websocket_thread; 
  
  if (pthread_mutex_init(&frameread_lock, NULL) != 0) { 
      printf("\n mutex init has failed\n"); 
      return -1; 
  } 

  if (gpioInitialise() < 0) return -1;
  gpioSetSignalFunc(SIGINT, INThandler);

  xn::pio::Arducam cam;
  cam.setQuality(4);
  // xn::vec2 cam_res = cam.get_resolution(); 
  // xn::vec2 cam_mid = {cam_res.x/2, cam_res.y/2};
  // int cam_dead_zone_x = cam_res.x*0.1;
  // int cam_dead_zone_y = cam_res.y*0.1;

  /* 
    cam widths: up/dow     : pin 24, w 705
                left/right : pin 23, w 1675
  */

  servo_cam_x.setWidth(servo_cam_origin.x);
  servo_cam_y.setWidth(servo_cam_origin.y);


  xn::vec3 pole = {0, 0, 1};
  xn::Transform bonechain[] = {
    xn::Transform({0, 0, 0}),
    xn::Transform({0, 1, 0}),
    xn::Transform({0, 1 + 6.9/17.2, 0}),
    xn::Transform({0, 1 + 13.8/17.2, 0})
  };
  float start_bone_lengths[] = {
    xn::vec3::dist(bonechain[0].position, bonechain[1].position),
    xn::vec3::dist(bonechain[1].position, bonechain[2].position)
  };

  xn::ik::IkChain arm(4, bonechain, pole);
  
  // phys ik test code
  std::vector<xn::pio::SmoothServo> servos[] = {
    std::vector<xn::pio::SmoothServo>(),
    std::vector<xn::pio::SmoothServo>(),
    std::vector<xn::pio::SmoothServo>(),
    std::vector<xn::pio::SmoothServo>()
  };

  servos[0].push_back(xn::pio::SmoothServo(xn::pio::ServoAngular(12, 500, 2500), {0,1,0}, bonechain[0].position, 1400 ));
  servos[0].push_back(xn::pio::SmoothServo(xn::pio::ServoAngular(13, 500, 2500), {1, 0,0}, bonechain[0].position, 1600 ));
  servos[1].push_back(xn::pio::SmoothServo(xn::pio::ServoAngular(26, 500, 2500), {1, 0,0}, bonechain[1].position, 1600 ));
  servos[2].push_back(xn::pio::SmoothServo(xn::pio::ServoAngular(16, 500, 2500), {-1,0,0}, bonechain[2].position, 1500 ));

  xn::pio::SmoothServo wrist(xn::pio::ServoAngular(6, 500, 2500));
  // wrist.target_angle = M_PI;
  wrist.servo.setAngle(M_PI_2);
  wrist.t_max *= 0.5;
  wrist.target_angle = M_PI/2 ;
  pthread_create(&wrist.tid, NULL, xn::ik::move_servo_thread, &wrist);
  wristptr = &wrist;

  xn::pio::SmoothServo claw(xn::pio::ServoAngular(5,500,2500));
  claw.servo.setAngle(claw_open_a);
  claw.target_angle = claw_open_a;
  pthread_create(&claw.tid, NULL, xn::ik::move_servo_thread, &claw);
  clawptr = &claw;

  xn::ik::ServoChain phys_arm(arm, servos, &run, 172 );
  phys_arm.servos[0][0].t_max *= 0.5;
  armptr = &phys_arm;

  phys_arm.servos[0][1].servo.setAngle(M_PI/2);
  phys_arm.servos[1][0].servo.setAngle(0.1);
  phys_arm.servos[1][0].target_angle = M_PI/2;
  phys_arm.servos[2][0].servo.setAngle(M_PI);
  phys_arm.servos[2][0].target_angle = M_PI/2;

  // host webapp for manual servo control
  // TODO: send camera feed to clients
  init_msg_q();
  websocket_thread = ws_create_thread("www.studiostudios.net", 3000);

  // open socket for video stream
  struct sockaddr_in serv_addr; 
  if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) { 
      printf("socket() failed:\n"); 
      return -1; 
  } 
  serv_addr.sin_family = AF_INET; 
  serv_addr.sin_port = htons(laptop_port);   
  if(inet_pton(AF_INET, laptop_ip, &serv_addr.sin_addr)<=0) { 
      printf("Invalid address\n"); 
      return -1; 
  } 

  // blocks until accepted
  if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)  { 
      printf("connect() failed\n"); 
      return -1; 
  } 

  start_read_data = 1;
  pthread_create(&cam_io_tid, NULL, cam_io_thread, (void*) &cam); // send camera feed to laptop for processing
  pthread_create(&servo_ctl_tid, NULL, servo_ctl_thread, NULL); // recv servo commands from laptop


  // while(receiving_commands)
  // {
  //     int p = 23;
  //     int t = 1500;
  //     // float t = 0.5;
  //     printf("\npin width: ");
  //     std::cin >> p >> t;
  //     // serv.setWidth(t);
  //     gpioServo(p, t);
  // }

  while(receiving_commands)
  {
      if ( move_to_target ) {
        phys_arm.grab_safe(wristlen, target);
      }

      phys_arm.move_wrist(wrist);
  }

  pthread_join(websocket_thread, NULL); 
  pthread_join(cam_io_tid, NULL);
  pthread_join(servo_ctl_tid, NULL); 

  if ( sock )
    close(sock);
  start_read_data = 0;
  pthread_mutex_destroy(&frameread_lock); 
  printf("\nlater nerd\n");
  return 0;
}

void read_int32_t(int sock, int32_t *intptr ) {
      int32_t tmp; 
      uint32_t left = sizeof(*intptr);
      char *buf = (char *) &tmp;
      int rc;
      do {
        rc =  recv(sock, buf, left, 0);
        buf += rc;
        left -= rc;
      } while (left > 0);
      *intptr = ntohl(tmp);
}

void send_int32_t(int sock, int32_t val) {
    int32_t nval = htonl(val);
    int bs = send(sock, (const char*)&nval, sizeof(nval), 0);
    assert(bs == sizeof nval);
}

void *servo_ctl_thread(void *arg) {
  auto time_last_seen = xn::pio::get_time();
  while (receiving_commands) {
    if ( sock ) {

      int32_t x, y, z, fx, fy, fz, ofx, ofy, ofz;
      read_int32_t(sock, &x);
      read_int32_t(sock, &y);
      read_int32_t(sock, &z);

      read_int32_t(sock, &ofx);
      read_int32_t(sock, &ofy);
      read_int32_t(sock, &ofz);

      // move in front of target
      target = {-x, 30,-y};
      DUMP(target);
      wristlen = 150;
      wristptr->t_max = armptr->servos[0][1].t_max/4;
      move_to_target = true;
      time_sleep(armptr->servos[0][1].t_max/2);

      // move to target
      wristptr->t_max = armptr->servos[0][1].t_max*0.75;
      wristlen = 80;
      time_sleep(armptr->servos[0][1].t_max/4);

      // grab
      clawptr->target_angle = claw_close_a;
      time_sleep(armptr->servos[0][1].t_max/4);
      
      // move to neutral position
      move_to_target = false;
      wristptr->t_max = armptr->servos[0][1].t_max/2;\
      armptr->stiffy();

      // bool found = false;
      face_found = false;
      cam_ang_orig.x = servo_cam_x.getWidth();
      cam_ang_orig.y = servo_cam_y.getWidth();
      servo_cam_y.setWidth(1500);
      send_int32_t(sock, -1);
      pthread_t srch_tid;
      pthread_create(&srch_tid, NULL, cam_search_thread, NULL );
      read_int32_t(sock, &fx);
      read_int32_t(sock, &fy);
      read_int32_t(sock, &fz);
      send_int32_t(sock, -2);

      DUMP(fx);
      DUMP(fy);
      DUMP(fz);

      face_found = true;
      pthread_join(srch_tid, NULL);

      xn::vec3 facet{fx, fy, fz};
      facet = facet*10;
      DUMP(cam_ang_diff.y * M_PI / 2000);
      DUMP(cam_ang_diff.x * M_PI / 2000);
      facet = xn::vec3::rotate_axis(facet, {1,0,0},  cam_ang_diff.y * (M_PI + M_PI/4)  / 2000);
      facet = xn::vec3::rotate_axis(facet, {0,-1, 0}, cam_ang_diff.x *(M_PI + M_PI/4)  / 2000);
      xn::vec3 of = { -ofx, -ofz, -ofy};
      facet = facet + of;
      target = {facet.x, facet.y, facet.z};
      wristlen = 10;
      // target = target * 
      move_to_target = true;

// target = (-95.6016, 369.345, 244.609)

      DUMP(target);
      DUMP(of);

      time_sleep(armptr->servos[0][1].t_max*0.75);
      move_to_target = false;
      
      // release
      clawptr->target_angle = claw_open_a;
      time_sleep(armptr->servos[0][1].t_max/4);
      
      armptr->stiffy();
      servo_cam_x.setWidth(servo_cam_origin.x);
      servo_cam_y.setWidth(servo_cam_origin.y);


    }
  } 
  return NULL;
}

void *cam_io_thread(void *arg) {
  xn::pio::Arducam cam = *(xn::pio::Arducam *) arg;
  while (1) {
    if (start_read_data == 1) {
      auto t1 = xn::pio::get_time();
      cam.capture();
      // printf("time to capture: %f\n", xn::pio::time_diff_seconds(t1, xn::pio::get_time()));

      // send frame over TCP socket
      if (sock) { 
        // send buffer size 1st cuz jpeg compression makes the size unpredictable 
        int32_t send_len = cam.frame_size;
        int32_t tmp = htonl(send_len);
        send(sock, &tmp, sizeof(tmp), 0);

        // send frame buffer
        size_t bs = send(sock, cam.frame_buffer, cam.frame_size, 0);

        // printf("frame_len = %d\n", cam.frame_size);
        // printf("bs = %d\n", bs);

        assert(bs == cam.frame_size);
      }       
    }
  }
  return NULL;
}

void  INThandler(int sig) {
  signal(sig, SIG_IGN); 
  
  receiving_commands = false;
  interrupted = false;
  time_sleep(0.1);
  printf("\nmoving servos back to neutral\n");

  armptr->curl();
  wristptr->target_angle = 0.1;
  armptr->servos[0][0].target_angle = M_PI_2;
  
  time_sleep(armptr->servos[0][1].t_max/2);	
  run = false;
  exit(0);
}

void *cam_search_thread(void *argv) {
    while ( !face_found ) {
      int step_width = 150;
      static int dir = -1;

      int w = step_width*dir;
      if ( servo_cam_x.getWidth()+ w > servo_cam_x.max_width || servo_cam_x.getWidth()+ w < servo_cam_x.min_width ) {
        dir *= -1;
        w = step_width*dir;
      }
      
      servo_cam_x.setWidth(servo_cam_x.getWidth()+ w);

      cam_ang_diff.x = (servo_cam_x.getWidth() - cam_ang_orig.x);
      cam_ang_diff.y = (servo_cam_y.getWidth() - servo_cam_y_perp_w);
      // DUMP(cam_ang_diff.x);
      // DUMP(cam_ang_diff.y);
      time_sleep(1.5);
  }
}