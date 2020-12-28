// #define PIO_VIRTUAL

#include "xn_ik.hpp"
#include "assert.h"
#include <cmath>

#include <fenv.h>

using namespace xn;
using namespace std;


int run=1;

void stop(int signum) { run = 0; }

int main() {
  feenableexcept(FE_INVALID | FE_OVERFLOW);

#ifndef PIO_VIRTUAL  
  if (gpioInitialise() < 0) return -1;
   gpioSetSignalFunc(SIGINT, stop);
#endif
  // vec3 target{0,1.4, 0};

  vec3 pole = {0, 0, 1};
  Transform bonechain[] = {
    Transform({0, 0, 0}),
    Transform({0, 1, 0.3}),
    Transform({0, 2, 0})
  };
  float start_bone_lengths[] = {
    vec3::dist(bonechain[0].position, bonechain[1].position),
    vec3::dist(bonechain[1].position, bonechain[2].position)
  };

  ik::IkChain arm(3, bonechain, pole);
  

  // phys ik test code
  vector<pio::ServoAngular> servos[] = {
    vector<pio::ServoAngular>(),
    vector<pio::ServoAngular>(),
    vector<pio::ServoAngular>()
  };
  servos[0].push_back(pio::ServoAngular(26, 500, 2500));
  servos[0].push_back(pio::ServoAngular(16, 575, 2460));
  servos[1].push_back(pio::ServoAngular(6, 575, 2460));

  printf("%f %f\n", servos[0][1].min_angle, servos[0][1].max_angle);

  vector<vec3> axes[] = {
    vector<vec3>(),
    vector<vec3>(),
    vector<vec3>(),
  };

  axes[0].push_back({0,1,0});
  axes[0].push_back({1,0,0});
  axes[1].push_back({1,0,0});

  ik::ServoChain phys_arm(arm, servos, axes );
  phys_arm.reset();
  // time_sleep(0.5);
  start_bone_lengths[0] = vec3::dist(phys_arm.positions[0], phys_arm.positions[1]);
  start_bone_lengths[1] = vec3::dist(phys_arm.positions[1], phys_arm.positions[2]);

 
  float tot_err = 0;
  float avg_err = 0;
  int iters = 0;
  // while ( run ) {
  for ( int j = 0; j < 100; j++) {
    static float i = 0;
    static float iter = 0.01;

    if (i+iter > 1|| i+iter < 0 )
      iter*=-1;
    
    i+= iter;

    static float ang = 0;
    // ang = fmod(ang+0.01, M_PI);
    ang = ang+0.01;
    vec3 target{cosf(ang)*10, 10, sinf(ang)*10};
    // vec3 target{0, 1.3+ i, 0};
    // vec3 target{-1+i*2 , 1, 1};

    vec3 pole{0, 0, 1};
    // vec3 pole{-1+i*2 , 1, 1};
    // printf("pole = ");
    // pole.print();
    Transform bonechain[] = {
      Transform({0, 0, 0}),
      Transform({0, 1, 0.01}),
      Transform({0, 2, 0})
    };
    float start_bone_lengths[] = {
      vec3::dist(bonechain[0].position, bonechain[1].position),
      vec3::dist(bonechain[1].position, bonechain[2].position)
    };

    ik::IkChain arm(3, bonechain, pole);
    phys_arm.ideal_chain = arm;

    printf("target = ");
    target.print();    
    phys_arm.resolve(target);
    // cout << "bone positions\n";
    // for ( int i = 0; i < phys_arm.ideal_chain.bone_count; i++ ) {
    //   cout << phys_arm.positions[i] << '\n'; 
    // }

    tot_err += vec3::dist(phys_arm.positions[2], phys_arm.ideal_chain.positions[2]);
    avg_err = tot_err / (++iters);
#ifndef PIO_VIRTUAL
    time_sleep(0.01);
#endif
  }
  
  printf("\n");
  DUMP(avg_err);

  std::string dummy;
  std::cout << "Enter to continue..." << std::endl;
  std::getline(std::cin, dummy);
}