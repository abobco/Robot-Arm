#pragma once

#include <cmath>
#include <stdio.h>
#include <iostream>
#include <vector>
#include "xn_gpio.hpp"
#include <pthread.h>
#include "xn_vec.hpp"

#ifdef PIO_VIRTUAL
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#endif

bool run=1;

namespace xn 
{ 
    struct Transform {
        struct Transform *parent;
        vec3 position;
        vec3 eulers;

        Transform() {
            parent = NULL;
            position = { 0,0,0};
            eulers = {0,0,0};
        }

        Transform(vec3 _position, vec3 _eulers = {0,0,0}) {
            parent = NULL;
            position = _position;
            eulers = _eulers;
        }
    };

    namespace ik 
    {
        // cyclic coordinate descent inverse kinematics solver w/ poles 
        class IkChain 
        {
        public:
            int bone_count;
            int iterations;         // max iterations
            float delta = 0.001;    // max distance to target for early success

            float *bone_lengths;
            float chain_length = 0;
            Transform *bones;
            vec3 *positions;
            vec3 pole_main;
            vec3 *poles;

            IkChain() {}

            IkChain(int _bone_count, Transform ordered_bones[], vec3 &_pole, int _iterations = 10)  {
                bone_count = _bone_count;
                iterations = _iterations;
                pole_main = _pole;

                bones = new Transform[bone_count +1];
                bone_lengths = new float[bone_count +1];
                positions = new vec3[bone_count +1];
                poles = new vec3[bone_count-1];

                for ( int i = 0; i < bone_count; i++ ) {
                    bones[i] = ordered_bones[i];
                }
                for ( int i = 1; i < bone_count; i++ ) {
                    bones[i].parent = &bones[i-1]; 
                }

                Transform current = bones[bone_count-2];
                for ( int i = 0; i < bone_count; i++ ) {
                    bone_lengths[i] = 0;
                }

                for ( int i = bone_count-2; i>=0; i-- ) {
                    // bone_start_rotations[i] = current.rotation;
                    bone_lengths[i] = vec3::dist(bones[i+1].position, bones[i].position);
                    // printf("bone_lengths[%d] = %f\n", i, bone_lengths[i]);
                    chain_length += bone_lengths[i];
                }
            }

            void resolve(vec3& target) {
                // save a copy of each bone position
                for ( int i = 0; i < bone_count; i++ ) {
                    positions[i] = bones[i].position;
                }

                if ( vec3::dist_sqr(target, bones[0].position ) > chain_length*chain_length ) {
                    // target is out of reach
                    vec3 dir = target - bones[0].position;
                    dir.normalize();
                    
                    for ( int i =1; i < bone_count; i++ ) {
                        positions[i] = positions[i-1] + dir*bone_lengths[i-1];
                        bones[i].position = positions[i];
                    }
                } else {
                    for ( int iter = 0; iter < iterations; iter++ ) { 
                        // top->bottom traversal, move joints towards target
                        for ( int i = bone_count-1; i > 0; i-- ) {
                            if ( i == bone_count-1 ) {
                                positions[i] = target;
                            } else {
                                vec3 directionChildToCurrentBone = positions[i] - positions[i+1];
                                directionChildToCurrentBone.normalize();
                                vec3 d = directionChildToCurrentBone * bone_lengths[i];
                                positions[i] = positions[i+1] + d;
                            }
                        }

                        // bottom->top traversal, move joints towards parent bones
                        for ( int i = 1; i < bone_count; i++ ) {
                            vec3 directionParentToCurrentBone = positions[i] - positions[i-1];
                            directionParentToCurrentBone.normalize();
                            vec3 d = directionParentToCurrentBone * bone_lengths[i-1];
                            positions[i] = positions[i-1] + d;
                        }

                        // early success
                        if ( vec3::dist_sqr(positions[bone_count-1], target) < delta*delta )
                            break;    
                    }

                    // resolve poles
                    for ( int i = 0; i < bone_count-2; i++) {
                        vec3& par = positions[i];
                        vec3& mid = positions[i+1];
                        vec3& child = positions[i+2];
                        
                        vec3 norm = par - child; // axis of rotation
                        norm.normalize();

                        // TODO: solve for remaining poles, follow a curve or sphere around the arm
                        if ( i > 0 )
                            poles[i] = {0,3,0}; 
                        else
                            poles[i] = pole_main;
                        
                        vec3 proj_mid = mid - norm * vec3::dot(mid, norm);
                        vec3 proj_pole = poles[i] - norm* vec3::dot(poles[i], norm);
                        
                        proj_mid = proj_mid - par;
                        proj_pole = proj_pole - par;

                        vec3 y_ax = vec3::cross(norm, proj_mid);
                        float x = vec3::dot(proj_pole, proj_mid);
                        float y = vec3::dot(proj_pole, y_ax);
                        
                        float ang = atan2f(y, x);

                        // rotate middle joint
                        vec3 relpos = mid - par;
                        positions[i+1] = par + vec3::rotate_axis(relpos, norm, ang);
                    }
                    
                    for ( int i = 0; i < bone_count; i++) {
                        bones[i].position = positions[i];
                    }
                }

            }

            void print() {
                for ( int i = 0; i < bone_count; i++ ) {
                    printf("bone[%d] = ", i);
                    bones[i].position.print();
                    printf("\tbone length: %f\n", bone_lengths[i] );
                    if ( bones[i].parent != NULL) {
                        Transform p = *bones[i].parent;
                        printf("\tparent = ");
                        p.position.print();
                    } else {
                        printf("\troot of chain\n");
                    }
                }
            }

            void reset() {
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

                *this = xn::ik::IkChain(4, bonechain, pole);
            }
        };
        
        void *move_servo_thread(void *argv) {
            pio::SmoothServo *s = (pio::SmoothServo*) argv;
            double step = 1.0/120;

            while ( run ) {
                // asymptotic lerp
                // s->servo.setAngle(s->servo.getAngle()*0.95 + s->target_ang*0.05);

                // bezier curve
                s->update(step);
                time_sleep(step);
            }
            return NULL;
        }

        // top level arm servo controller
        class ServoChain {
        public:
            IkChain ideal_chain;
            std::vector<pio::SmoothServo> *servos;

            vec3 *positions;
            float arm_len;
            
            ServoChain(IkChain bonechain, std::vector<pio::SmoothServo> _servos[], bool* loopvar, float _arm_len = 1 ) {      
                ideal_chain = bonechain;
                arm_len = _arm_len;

                servos = _servos;
                for ( int i = 0; i < bonechain.bone_count; i++ ) {
                    for (pio::SmoothServo& s: servos[i]) {
                        pthread_create(&s.tid, NULL, &move_servo_thread, &s);
                    }
                }
                
                positions = new vec3[ideal_chain.bone_count];
                for ( int i = 0; i < ideal_chain.bone_count; i++ ) 
                    positions[i] = bonechain.bones[i].position;
            }

            ~ServoChain() {
                for ( int i = 0; i < ideal_chain.bone_count; i++ ) {
                    for (pio::SmoothServo s : servos[i]) {
                        pthread_join(s.tid, NULL);
                    }
                }
            }

            void resolve(vec3& target) {
                vec3& pax = servos[0][0].axis;
                vec3 target_normalized = target / arm_len;
                vec3 pole = target_normalized - (pax * vec3::dot(target_normalized, pax) );
                if (pole.mag() < SMEPSILON )
                    pole = {3,0,0};
                else {
                    pole.normalize();
                    pole = pole*-5;
                }

                ideal_chain.pole_main = pole;
                ideal_chain.resolve(target_normalized);
                // printf("ideal chain:\n");
                // ideal_chain.print();
                for ( int i = 0; i < ideal_chain.bone_count-1; i ++ ) {
                    for ( int j = 0; j < servos[i].size(); j++ ) {
                        vec3 next_target = ideal_chain.bones[i+1].position - positions[i];
                        vec3 pos = positions[i+1] - positions[i];
                        vec3& ax = servos[i][j].axis;
                        vec3 proj_pos = pos - (ax * vec3::dot(pos, ax));
                        vec3 proj_target = next_target - (ax * vec3::dot(next_target, ax));

                        if ( proj_pos.mag() < SMEPSILON ) continue;

                        proj_pos.normalize();

                        vec3 y_ax = vec3::cross(ax, proj_pos);
                        float x = vec3::dot(proj_target, proj_pos);
                        float y = vec3::dot(proj_target, y_ax);
                        float ang = atan2f(y, x);
                        float s_ang;
                        if ( servos[i].size()>1 && j == 0)
                            s_ang = fmod(servos[i][j].target_angle + ang, servos[i][j].servo.max_angle);
                        else 
                            s_ang = clamp<float>(servos[i][j].target_angle + ang, servos[i][j].servo.min_angle, servos[i][j].servo.max_angle);
                        
                        if ( s_ang < 0 ) {
                            s_ang = servos[i][j].servo.max_angle + s_ang;
                        }
                        ang = s_ang - servos[i][j].target_angle;

                        servos[i][j].target_angle = s_ang;
                        // servos[i][j].servo.setAngle(s_ang);

                        // rotate servos in child joints
                        for ( int k = i+1;  k < ideal_chain.bone_count; k++ ) {
                            vec3 relpos = positions[k] - positions[i];
                            positions[k] = positions[i] + vec3::rotate_axis(relpos, ax, ang);
                            for ( int l = 0; l < servos[k].size(); l++ ) {
                                servos[k][l].axis = vec3::rotate_axis(servos[k][l].axis, ax, ang);
                                servos[k][l].axis.normalize();
                            }
                        }

                        // rotate servos axes in current joint
                        for ( int k = j+1; k < servos[i].size(); k++ ) {
                            servos[i][k].axis = vec3::rotate_axis(servos[i][k].axis, ax, ang);
                            servos[i][k].axis.normalize();
                        }
                    }
                }
            }

            void curl() {
                servos[0][1].target_angle = M_PI/2;
                servos[1][0].target_angle = 0;
                servos[2][0].target_angle = M_PI;
                servos[2][0].target_angle = M_PI;
            }

            void stiffy() {
                curl();
                time_sleep(servos[0][0].t_max/2);
                for ( int i = 0; i < ideal_chain.bone_count; i++ ) {
                    for ( pio::SmoothServo& s : servos[i] ) {
                        s.target_angle = M_PI/2;
                    }
                }
                time_sleep(servos[0][0].t_max/2);
                reset();
            }

            // solve for a point on the line from arm to target, wristlen away from the target
            void grab_safe(double wristlen, vec3& target, const vec3& arm_pos = {60, 0, -22} ) {
                xn::vec3 reltarg = target - arm_pos;
                
                // similar triangles solve for x & y offset of wrist 
                double d_armpos = sqrt(reltarg.x * reltarg.x + reltarg.z * reltarg.z);
                double dx = wristlen / (d_armpos / reltarg.x);
                double dz = wristlen / (d_armpos / reltarg.z);
                reltarg.x -= dx;
                reltarg.z -= dz;
                
                resolve(reltarg);
            } 

            void move_wrist(pio::SmoothServo& wrist) {
                float ac = 0;
                for ( int i = 0; i < ideal_chain.bone_count; i++ ) {
                    for ( xn::pio::SmoothServo& s : servos[i] ) {
                        if (s.axis.y > 0.5 )
                            continue;
                        float fac = s.target_angle - M_PI/2;
                        if ( i != 2 ) fac *= -1;
                        ac += fac;
                    }
                }

                wrist.target_angle = M_PI - ac;
            }

            // TODO: save start values from constructor instead of this bs
            void reset() {
                servos[0][0].position = {0, 0, 0};
                servos[0][1].position = {0, 0, 0};
                servos[1][0].position = {0, 1, 0};
                servos[2][0].position = {0, 1+ 6.9/17.2, 0};

                servos[0][0].axis = {0,1,0}; 
                servos[0][1].axis = {1, 0,0};
                servos[1][0].axis = {1, 0,0};
                servos[2][0].axis = {-1,0,0};

                positions[0] = {0, 0, 0};
                positions[1] = {0, 1, 0};
                positions[2] = {0, 1+ 6.9/17.2, 0};
                positions[3] = {0, 1+ 13.8/17.2, 0};
            }
        };
    }
}