// Copyright 2021 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cstdio>
#include <cstring>
#include <iostream>
#include <chrono>
#include <thread>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
#include <iostream>


// MuJoCo data structures
mjModel* m = NULL;                  // MuJoCo model
mjData* d = NULL;                   // MuJoCo data
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;
double PI = 3.14;
double thrust_adj[4] = {0, 0, 0, 0};
bool up = false, down = false, left = false, right = false;


double MAX_VEL = 0.05;

// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods) {
  // backspace: reset simulation
  if (act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE) {
    mj_resetData(m, d);
    mj_forward(m, d); // does not use integration - does not update position/velocity of points, but does update stuff like forces?
  }

  if (key == GLFW_KEY_W) {
    if(act == GLFW_PRESS) {
      printf("\n---------------up------------------");
      up = true;

      thrust_adj[0] -= 0.0001;
      thrust_adj[1] -= 0.0001;
      thrust_adj[2] += 0.0001;
      thrust_adj[3] += 0.0001;


      printf("%f", thrust_adj[2]);
    }else if (act == GLFW_RELEASE) {
      up = false;

    }
  }

  else if(act==GLFW_PRESS && key==GLFW_KEY_S) {
    printf("\n---------------S------------------");
    thrust_adj[0] += 0.0001;
    thrust_adj[1] += 0.0001;
    thrust_adj[2] -= 0.0001;
    thrust_adj[3] -= 0.0001;
  }
  else if(act==GLFW_PRESS && key==GLFW_KEY_A) {
    printf("\n---------------A------------------");
    thrust_adj[1] += 0.0001;
    thrust_adj[2] += 0.0001;

    thrust_adj[0] -= 0.0001;
    thrust_adj[3] -= 0.0001;
  }
  else if(act==GLFW_PRESS && key==GLFW_KEY_D) {
    printf("\n---------------D------------------");
    thrust_adj[0] += 0.0001;
    thrust_adj[3] += 0.0001;

    thrust_adj[1] -= 0.0001;
    thrust_adj[2] -= 0.0001;
  }
}


// mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods) {
  // update button state
  button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
  button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
  button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

  // update mouse position
  glfwGetCursorPos(window, &lastx, &lasty);
}


// mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos) {
  // no buttons down: nothing to do
  if (!button_left && !button_middle && !button_right) {
    return;
  }

  // compute mouse displacement, save
  double dx = xpos - lastx;
  double dy = ypos - lasty;
  lastx = xpos;
  lasty = ypos;

  // get current window size
  int width, height;
  glfwGetWindowSize(window, &width, &height);

  // get shift key state
  bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                    glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

  // determine action based on mouse button
  mjtMouse action;
  if (button_right) {
    action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
  } else if (button_left) {
    action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
  } else {
    action = mjMOUSE_ZOOM;
  }

  // move camera
  mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
}


// scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset) {
  // emulate vertical mouse motion = 5% of window height
  mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}


// main function
int main(int argc, const char** argv) {
  using namespace std::this_thread; // sleep_for, sleep_until
  using namespace std::chrono; // nanoseconds, system_clock, seconds

//   // check command-line arguments
//   if (argc!=2) {
//     std::printf(" USAGE:  basic modelfile\n");
//     return 0;
//   }

//   // load and compile model
//   char error[1000] = "Could not load binary model";
//   if (std::strlen(argv[1])>4 && !std::strcmp(argv[1]+std::strlen(argv[1])-4, ".mjb")) {
//     m = mj_loadModel(argv[1], 0);
//   } else {
//     m = mj_loadXML(argv[1], 0, error, 1000);
//   }

  char error[1000] = "Could not load binary model";

  m = mj_loadXML("/Users/warisz/Code/robasic/model/skydio_x2/scene.xml", 0, error, 1000);

  if (!m) {
    mju_error("Load model error: %s", error);
  }

  // make data
  d = mj_makeData(m);

  // init GLFW
  if (!glfwInit()) {
    mju_error("Could not initialize GLFW");
  }

  // create window, make OpenGL context current, request v-sync
  GLFWwindow* window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  // initialize visualization data structures
  mjv_defaultCamera(&cam);
  mjv_defaultOption(&opt);
  mjv_defaultScene(&scn);
  mjr_defaultContext(&con);

  // create scene and context
  mjv_makeScene(m, &scn, 2000);
  mjr_makeContext(m, &con, mjFONTSCALE_150);

  // install GLFW mouse and keyboard callbacks
  glfwSetKeyCallback(window, keyboard);
  glfwSetCursorPosCallback(window, mouse_move);
  glfwSetMouseButtonCallback(window, mouse_button);
  glfwSetScrollCallback(window, scroll);


  // printf("%f", cam.distance);
  cam.azimuth = 180;
  cam.distance = 10;
  // cam.lookat[2] = 2;

  double integral_sum = 0;
  double z_last_error = 0;

  double desired_z = 3;
  double desired_y = 0;
  double desired_x = 0;

  // run main loop, target real-time simulation and 60 fps rendering
  while (!glfwWindowShouldClose(window)) {
    // advance interactive simulation for 1/60 sec
    //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
    //  this loop will finish on time for the next frame to be rendered at 60 fps.
    //  Otherwise add a cpu timer and exit this loop when it is time to render.
    mjtNum simstart = d->time;

     while (d->time - simstart < 1.0/60.0) {
      // ref = 1m
       double curr_z = d->geom_xpos[17];
       double curr_y = d->geom_xpos[16];
       double curr_x = d->geom_xpos[15];

       printf("%f\n", curr_x);
       // std::cout << curr_height << std::endl;
       // std::cout << d->qvel[2] << std::endl;

       double z_error = desired_z - curr_z;
       double y_error = desired_y - curr_y;
       double x_error = desired_x - curr_x;


       integral_sum += z_error/60;

       // if(up == true) {
       //   desired_y += MAX_VEL * (1/60);
       //   // thrust_adj[0] -= 0.0001;
       //   // thrust_adj[1] -= 0.0001;
       //   // thrust_adj[2] += 0.0001;
       //   // thrust_adj[3] += 0.0001;
       // }

       // 2 0.5 0.5
       double K_p = 3;
       double K_i = 0;
       double K_d = 10000;

       double base_thrust = K_p*z_error + K_i*integral_sum + K_d*(z_error - z_last_error)/60;

       // printf("%f\n", curr_x);
       // printf("%f\n\n", last_error);

       // thrust_adj[0] = 0.01*y_error;
       // thrust_adj[3] = 0.01*y_error;
       // thrust_adj[2] = -K_p*y_error;
       // thrust_adj[3] = -K_p*y_error;

       z_last_error = z_error;

       // printf("%f", thrust_adj[0]);
       d->ctrl[0] = base_thrust + thrust_adj[0];
       d->ctrl[1] = base_thrust + thrust_adj[1];
       d->ctrl[2] = base_thrust + thrust_adj[2];
       d->ctrl[3] = base_thrust + thrust_adj[3];

      mj_step(m, d);
    }

    // get framebuffer viewport
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

    // update scene and render
    mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);


    mjr_render(viewport, &scn, &con);

    // swap OpenGL buffers (blocking call due to v-sync)
    glfwSwapBuffers(window);

    // process pending GUI events, call GLFW callbacks
    glfwPollEvents();
  }

  //free visualization storage
  mjv_freeScene(&scn);
  mjr_freeContext(&con);

  // free MuJoCo model and data
  mj_deleteData(d);

  // terminate GLFW (crashes with Linux NVidia drivers)
#if defined(__APPLE__) || defined(_WIN32)
  glfwTerminate();
#endif

  return 1;
}
