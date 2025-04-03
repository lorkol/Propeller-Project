#include "simpletools.h"                      // Include simple tools
#include "string.h"
#include <stdbool.h>
#include "adcDCpropab.h"   // Include ADC library
#include "fdserial.h" 
#include <cmath>

#define MAX_COMMANDS 50
#define MAX_CMD_LENGTH 300
#define RESET_BTN 2

#define M_PI 3.1415926535897932384626433
#define LINK1_LENGTH 60.0
#define LINK2_LENGTH 60.0
#define SW 4
#define JOYSTICK_DEADZONE 0.2


//Servo Pins
#define SHOULDER_PIN 15
#define ELBOW_PIN 16
#define WRIST_PIN 17

//Servo constants
#define SERVO_ITERATIONS 20 //iterations needed for the servos to be able to reach any angle
#define SMALL_SERVO_MOVEMENT_ITERATIONS 2
static volatile servo_iterations = SERVO_ITERATIONS;
#define WRIST_UP 50.0
#define WRIST_DOWN 70.0
#define CONSTRAIN(x, lower, upper) ((x < lower) ? lower : ((x > upper) ? upper : x))
#define UPPER_SHOULDER_CONSTRAIN 180.0
#define LOWER_SHOULDER_CONSTRAIN 0.0
#define UPPER_ELBOW_CONSTRAIN 180.0
#define LOWER_ELBOW_CONSTRAIN 0.0

static volatile float shoulder_angle = 132.0;
static volatile float elbow_angle = 142.3;
static volatile int wrist_angle = WRIST_UP;

static volatile float cur_x = -100.0;
static volatile float cur_y = 40.0;
static volatile float target_x;
static volatile float target_y;
struct Command{
    float shoulder_command;
    float elbow_command;
    float wrist_command;
};

// Function prototypes
void itos(int num, char *str);
void concat(char *dest, char *src);
bool IK(float x, float y, float* theta1, float* theta2);
bool workspace_check(float x, float y);
void parseCoordinates(char* input, Command coordinates[MAX_COMMANDS], int* count);
void drawJoystickMap(float jx, float jy);
void update_joint_angles(float jx, float jy);
void servo_command(int pin, float angle);
void all_servos(Command command);
void init_servo_cogs();
void close_cogs();

void shoulder_movement(void *par);
void elbow_movement(void *par);
void wrist_movement(void *par);

static volatile Command current_command = {-1., -1., -1.};
static volatile bool shoulder_working = false;
static volatile bool elbow_working = false;
static volatile bool wrist_working = false;
unsigned int shoulder_stack[40+25];
unsigned int elbow_stack[40+25];
unsigned int wrist_stack[40+25];

//Variables for filler process
static volatile bool pen_down = false;
const int MAX_FILLER_COMMANDS = 2*(LINK1_LENGTH+LINK2_LENGTH);
struct Point{
    bool pen_up;
    bool pen_down;
    float x;
    float y;
    bool legitimate_point;
};
Point path[MAX_COMMANDS];
Command filler_path[MAX_FILLER_COMMANDS];
int path_filler(Point p1, Point p2);

// Add this helper function before all_servos
bool equal_float(float a, float b) {
    return fabs(a - b) < 0.1;  // Tolerance of 0.1 for one decimal place
}

int main() // Main function
{
  pause(1000);
  print("Program Started. Initializing...");
  init_servo_cogs();
  all_servos({shoulder_angle, elbow_angle, WRIST_UP});

  char* selection;
  while(1)
  {
    printf("Please choose program to use:\n");
    printf("- Pre-programmed drawing (P)\n");
    printf("- Joystick Control (J)\n");
    printf("- Draw Workspace (W)\n");
    scanf("%s", selection);
    if (!strcmp(selection,"J"))
    {
        adc_init(21, 20, 19, 18);  // CS=21, SCL=20, DO=19, DI=18

        float lrV, udV;  // Voltage variables
        
        
        while (!get_state(RESET_BTN)) {
            
            
            udV = adc_volts(2);  // Read Up/Down voltage
            lrV = adc_volts(3);  // Read Left/Right voltage   
                
            //drawJoystickMap(lrV, udV);  // Display joystick movement as a map
            //char* test;
            //itos(get_state(4),test);
            //print("%s\n",test);
            
            if ((lrV < (2.5-JOYSTICK_DEADZONE) || udV < (2.5-JOYSTICK_DEADZONE) || lrV > (2.5+JOYSTICK_DEADZONE) || udV > (2.5+JOYSTICK_DEADZONE))){
                update_joint_angles(lrV, udV);  // Update joint angles
                //print("lrV = %f, udV = %f\n", lrV, udV);
                print("joint angles updated: (%f,%f)\n\n", shoulder_angle, elbow_angle);
            }
            else if(get_state(4)){
              if(wrist_angle==WRIST_DOWN){
                all_servos({-1., -1., WRIST_UP});
                 wrist_angle=WRIST_UP;
              }
              else if(wrist_angle==WRIST_UP){
                all_servos({-1., -1., WRIST_DOWN});
                 wrist_angle=WRIST_DOWN;
              }
              pause(500);
            }               
            pause(100);
            
        }
    }
    else if(!strcmp(selection,"P"))
    {
        char input_str[MAX_CMD_LENGTH] = "-100,50 d -100,50 -93,73 u -96,63 d -96,63 -72,62 u -76,51 d -76,51 -68,77 u -63,98 d -63,98 -33,82 -31,113 -43,100 -63,98 u 7,85 d 7,85 12,116 29,83 36,112 u 56,105 d 56,105 59,90 52,68 59,90 74,93 u 90,77 d 90,77 73,60 90,43 103,56 u 73,30";
        Command commands[MAX_COMMANDS];
        int count = 0;
        Point prev_location;
        parseCoordinates(input_str, commands, &count);
        printf("num commands = %d\n", count);
        int steps = 0; //Num of steps for filler path
        for(int i = 0; i < count; i++)
        {
            print("command sent : %f, %f, %f\n", commands[i].shoulder_command, commands[i].elbow_command, commands[i].wrist_command);
            if(i>1)
            {
                print("current location = %f,%f\n", path[i].x, path[i].y);
                if (path[i-1].pen_up) pen_down = false;
                if (path[i-1].pen_down) pen_down = true;

                if(pen_down && !path[i].pen_up && path[i].legitimate_point)
                {
                    prev_location = path[i-1].pen_down ? path[i-2] : path[i-1];
                    print("prev location = %f, %f\n", prev_location.x, prev_location.y);
                    steps = path_filler(prev_location, path[i]);
                    servo_iterations = SMALL_SERVO_MOVEMENT_ITERATIONS;
                    for(int j = 0; j < steps; j++) all_servos(filler_path[j]);
                }
            }
            servo_iterations = SERVO_ITERATIONS;
            all_servos(commands[i]);
            if (get_state(RESET_BTN)) break;
            pause(10);
        }
    }
    else if(!strcmp(selection,"W"))
    {
        print("Drawing workspace...\n");
        all_servos({25., 90., -1.});
        all_servos({-1., -1., 80.});
        for(float i = 25.; i < 50.; i++)
        {
            all_servos({i, -1., -1.});
            //pause(100);
        }
        all_servos({-1., -1., WRIST_DOWN});
        for(float i = 50.; i < 160.; i++)
        {
            all_servos({i, -1., -1.});
            //pause(100);
        }
        all_servos({-1., -1., WRIST_UP});
        all_servos({130., 180., -1.});
        all_servos({-1., -1., WRIST_DOWN});
        for(int i = 130; i >= 50; i--)
        {
            all_servos({i, -1., -1.});
            //pause(100);
        }
        all_servos({-1., -1., WRIST_UP});
        all_servos({140., 0., -1.});
        all_servos({-1., -1., WRIST_DOWN});
        for(int i = 140; i >= 50; i--)
        {
            all_servos({i, -1., -1.});
            //pause(100);
        }
        all_servos({-1., -1., WRIST_UP});
        print("Workspace drawn\n");
    }
    else printf("Invalid input. Please enter P or J.\n");
 }

 close_cogs;
}

void parseCoordinates(char input[MAX_CMD_LENGTH], Command coordinates[MAX_COMMANDS], int* count) {
    *count = 0;
    char *token = strtok(input, " ");
    float x, y;
    float theta1, theta2;
    char buffer[MAX_CMD_LENGTH]; //Will contain 'angle,angle' so at most 4+1+4 chars
    bool success;

    while(token != NULL && *count < MAX_COMMANDS) {
        printf("token = %s\n",token);
        if(token[0] == 'u') 
        {
            coordinates[*count] = {-1., -1., WRIST_UP};
            path[*count] = {true, false, -666., -666., true};
        }
        else if(token[0] == 'd')
        {
            coordinates[*count] = {-1., -1., WRIST_DOWN};
            path[*count] = {false, true, -666., -666., true};
        }
        else 
        {
            // Split the token at the comma to get x and y coordinates
            char *comma_pos = strchr(token, ',');
            if(comma_pos != NULL) {
                // Temporarily replace comma with null terminator
                *comma_pos = '\0';
                x = atof(token);
                y = atof(comma_pos + 1);
                path[*count] = {false, false, x, y, true};
                // Restore the comma
                *comma_pos = ',';
            } 
            else {// Invalid format - do nothing
                coordinates[*count] = {-1., -1., -1.};
                path[*count] = {false, false, -666., -666., false};
                continue;
            } 
        success = workspace_check(x, y);
        if (success) success = IK(x, y, &theta1, &theta2);
        else print("Can not reach position %f,%f - it is outside of the workspace", x, y);

        //Round up theta1 and theta2 and convert to integers
        // theta1 = round(theta1);
        // theta2 = round(theta2);        
        
        if(success)
        {
          // Convert to integers

            shoulder_angle = theta1;
            elbow_angle = theta2;
            coordinates[*count] = {theta1, theta2, -1.};
            print("angles = %f,%f\n", theta1,theta2);
        }
        else// IK failed
            coordinates[*count] = {-1., -1., -1.};
      }

      (*count)++;
      token = strtok(NULL, " ");  // Get the next token
    }
}

bool IK(float x, float y, float* theta1, float* theta2) {
    float c2 = (x*x + y*y - LINK1_LENGTH*LINK1_LENGTH - LINK2_LENGTH*LINK2_LENGTH)/(2*LINK1_LENGTH*LINK2_LENGTH);

    // Print input coordinates first for debugging
    //print("IK for point: x=%f, y=%f\n", x, y);
          
    //print("c2 = %d.%d\n", (int)c2, (int)(c2*100)%100);

    if(c2 >= -1 && c2 <= 1 && y >= 0)
    {
        // float s2 = sqrt(1 - c2*c2);  // Positive solution TODO add negative solution when optimizing
        
        // float t1 = atan2(-LINK2_LENGTH*s2*x + (LINK1_LENGTH+LINK2_LENGTH*c2)*y,
        //                 (LINK1_LENGTH + LINK2_LENGTH*c2)*x + LINK2_LENGTH*s2*y);
        
        float alpha = acos((x*x + y*y + LINK1_LENGTH*LINK1_LENGTH - LINK2_LENGTH*LINK2_LENGTH)/(2*LINK1_LENGTH*sqrt(x*x + y*y)));
        float beta = acos((-x*x - y*y + LINK1_LENGTH*LINK1_LENGTH + LINK2_LENGTH*LINK2_LENGTH)/(2*LINK1_LENGTH*LINK2_LENGTH));
        float gamma = atan2(y,x);
        
        // Convert to integer degrees multiplied by 100 for printing
        //print("alpha=%f, beta=%f, gamma=%f\n", alpha, beta, gamma);
        
        if (x > 0)
        {
            *theta1 = gamma + alpha;
            *theta2 = beta-M_PI;
        }
        else if (x < 0)
        {
            *theta1 = gamma - alpha;
            *theta2 = M_PI - beta;
        }
        else //if x == 0
        {
          *theta1 = asin(y/(2*LINK1_LENGTH));
          *theta2 = M_PI - 2*(*theta1);
        }
        
        *theta1 = *theta1 * 180.0/M_PI;
        *theta2 = (*theta2 * 180.0/M_PI) + 90;
        print("theta1_deg=%f, theta2_deg=%f\n", *theta1, *theta2);
        return true;
    } 
    else 
    {
        // Point is unreachable
        printf("Could not reach point %f, %f - out of workspace\n", x, y);
        *theta1 = 0;
        *theta2 = 0;
        return false;
    }
}

bool workspace_check(float x, float y)
{
    return (LINK1_LENGTH - (LINK2_LENGTH - LINK1_LENGTH) <= sqrt(pow(x, 2) + pow(y, 2)) < LINK1_LENGTH + LINK2_LENGTH);
}

int path_filler(Point p1, Point p2)
{
    for(int k = 0; k < MAX_FILLER_COMMANDS; k++) filler_path[k] = {-1., -1., -1.};
    float dx = p2.x - p1.x;
    float dy = p2.y - p1.y;
    float shoulder_ang;
    float elbow_ang;
    int steps = ((int) (fabs(dx)<fabs(dy) ? fabs(dy) : fabs(dx))) + 1;
    float x_step_size = dx/steps;
    float y_step_size = dy/steps;
    print("steps = %d\n", steps);
    print("x_step_size = %f, y_step_size = %f\n", x_step_size, y_step_size);
    bool success;
    for(int i = 0; i < steps ; i++)
    {
        success = IK(p1.x + i*x_step_size, p1.y + i*y_step_size, &shoulder_ang, &elbow_ang);
        if(success) filler_path[i] = {shoulder_ang, elbow_ang, -1.};
        else filler_path[i] = {-1., -1., -1.};
    }
    return steps;
}

void itos(int num, char *str) {
    int i = 0;
    if (num == 0) {
        str[i++] = '0';
    } 

    while (num > 0) {
        str[i++] = (num % 10) + '0';
        num /= 10;
    }

    if (i > 0) {
        // Reverse the string
        for (int j = 0; j < i / 2; j++) {
            char temp = str[j];
            str[j] = str[i - j - 1];
            str[i - j - 1] = temp;
        }
    }

    str[i] = '\0'; // Null terminate the string
}

void concat(char *dest, char *src) {
    int i = 0;
    while (dest[i] != '\0') {
        i++;
    }

    int j = 0;
    while (src[j] != '\0') {
        dest[i++] = src[j++];
    }

    dest[i] = '\0'; // Null terminate the string
}

void drawJoystickMap(float vx, float vy) {
    char grid[5][5] = {
        {'-', '-', '-', '-', '-'},
        {'-', '-', '-', '-', '-'},
        {'-', '-', 'O', '-', '-'},  // Default center position
        {'-', '-', '-', '-', '-'},
        {'-', '-', '-', '-', '-'}
        };

    // Convert voltage (0V-4V) to grid coordinates (0-4)
    int gridx = (int)(vx);
    int gridy = (int)(vy);

    // Ensure values stay within 0-4 range
    if (gridx > 4) gridx = 4;
    if (gridy > 4) gridy = 4;

    // Invert Y-axis so UP is at the top of the grid
    gridy = 4 - gridy;

    // Place joystick position on the grid
    grid[gridy][gridx] = 'X';

    
    print("%c",HOME);  // Clear screen and move cursor to top-left
    //pause(100);
    print("\nJoystick Position:\n");

    // Print the 5x5 grid
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
            print("%c ", grid[i][j]);
        }
        print("\n");
    }
    print("%c",HOME);
}

void update_joint_angles(float vx, float vy) {
    // Convert voltage (0V-5V) to grid coordinates (-2 to 2)

    
    int xStep = (int)(vx);
    int yStep = (int)(vy);

    // // Ensure values stay within 0-4 range
    // if (xStep > 4) xStep = 4;
    // if (yStep > 4) yStep = 4;

    // // Invert Y-axis so UP is at the top of the grid
    // yStep = 4 - yStep;

    // Update joint angles based on joystick position. If xStep/yStep is 2 then no change. 
    // If xStep/yStep is 1 decrement shoulder_angle/elbow_angle by 1 degree.
    // If xStep/yStep is 0 decrement shoulder_angle/elbow_angle by 2 degrees.

    // X-axis controls X-position
    if(vx < 2.5-JOYSTICK_DEADZONE) {
        if(vx < 0.2) target_x = cur_x - 2;  // Far left: decrement by 2
        else target_x = cur_x - 1;          // Left: decrement by 1
    } else if(vx > 2.5+JOYSTICK_DEADZONE) {
            if(vx > 4.7) target_x = cur_x + 2;  // Far right: increment by 2
        else target_x = cur_x + 1;          // Right: increment by 1
    }

    // Y-axis controls Y-position
    if(vy < 2.5-JOYSTICK_DEADZONE) {
        if(vy < 0.2) target_y = cur_y - 2;     // Far down: decrement by 2
        else target_y = cur_y - 1;             // Down: decrement by 1
    } else if(vy > 2.5+JOYSTICK_DEADZONE) {
        if(vy > 4.7) target_y = cur_y + 2;     // Far up: increment by 2
        else target_y = cur_y + 1;             // Up: increment by 1
    }

    float target_shoulder_angle;
    float target_elbow_angle;

    bool success = IK(target_x, target_y, &target_shoulder_angle, &target_elbow_angle);

    if(!success){
        print("Could not reach target point %f, %f - out of workspace\n", target_x, target_y);
        return;
    }
    else{
        cur_x = target_x;
        cur_y = target_y;
        // Constrain angles between 0 and 180 degrees
        if(target_shoulder_angle < 0) target_shoulder_angle = 0.;
        if(target_shoulder_angle > 180) target_shoulder_angle = 180.;
        
        if(target_elbow_angle < 0) target_elbow_angle = 0.;
        if(target_elbow_angle > 180) target_elbow_angle = 180.;


        if (equal_float(target_shoulder_angle, shoulder_angle) && equal_float(target_elbow_angle, elbow_angle)) return;
        else if (equal_float(target_shoulder_angle, shoulder_angle) && !equal_float(target_elbow_angle, elbow_angle))
        {
            all_servos({-1., target_elbow_angle, -1.});
            elbow_angle = target_elbow_angle;
        }
        else if (!equal_float(target_shoulder_angle, shoulder_angle) && equal_float(target_elbow_angle, elbow_angle))
        {
            all_servos({target_shoulder_angle, -1., -1.});
            shoulder_angle = target_shoulder_angle;
        }
        else
        {
        all_servos({target_shoulder_angle, target_elbow_angle, -1.});
        shoulder_angle = target_shoulder_angle;
        elbow_angle = target_elbow_angle;
        }
        print("Target point %f, %f reached with angles %f, %f\n", target_x, target_y, target_shoulder_angle, target_elbow_angle);
    }

    
}

void init_servo_cogs()
{
    int kuku = cogstart(&shoulder_movement, NULL, shoulder_stack, sizeof(shoulder_stack));
    int koko = cogstart(&elbow_movement, NULL, elbow_stack, sizeof(elbow_stack));
    int coucou = cogstart(&wrist_movement, NULL, wrist_stack, sizeof(wrist_stack));
}

void close_cogs()
{

}

void all_servos(Command command)
{
    if((command.shoulder_command == current_command.shoulder_command && command.elbow_command == current_command.elbow_command && command.wrist_command == current_command.wrist_command)
     || (command.shoulder_command == -1. && command.elbow_command == -1. && command.wrist_command == -1.)) return;
    while(wrist_working || shoulder_working || elbow_working){};
    current_command.shoulder_command = command.shoulder_command;
    current_command.elbow_command = command.elbow_command;
    current_command.wrist_command = command.wrist_command;
    if(command.shoulder_command != -1.) shoulder_working = true;
    if(command.elbow_command != -1.) elbow_working = true;
    if(command.wrist_command != -1.) wrist_working = true;   
}

void shoulder_movement(void *par)
{
    while(1)
    {
        while(!shoulder_working) pause(20);
        servo_command(SHOULDER_PIN, CONSTRAIN(current_command.shoulder_command, LOWER_SHOULDER_CONSTRAIN, UPPER_SHOULDER_CONSTRAIN));
        shoulder_working = false;
    }
}

void elbow_movement(void *par)
{
    while(1)
    {
        while(!elbow_working) pause(20);
        servo_command(ELBOW_PIN, 180 - CONSTRAIN(current_command.elbow_command, LOWER_ELBOW_CONSTRAIN, UPPER_ELBOW_CONSTRAIN));
        elbow_working = false;
    }
}

void wrist_movement(void *par)
{
    while(1)
    {
        while(!wrist_working) pause(20);
        servo_command(WRIST_PIN, current_command.wrist_command);
        if(equal_float(current_command.wrist_command,WRIST_DOWN)) pen_down = true;
        if(equal_float(current_command.wrist_command,WRIST_UP)) pen_down = false;
        wrist_working = false;
    }
}


void servo_command(int pin, float angle){
    int pulse_width = (int)(500 + 10*angle);
    for (int i = 0; i < servo_iterations ; i++){
        pulse_out(pin, pulse_width);
        pause(20);
        }
}