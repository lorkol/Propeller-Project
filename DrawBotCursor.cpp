#include "simpletools.h"                      // Include simple tools
#include "string.h"
#include <stdbool.h>
#include "adcDCpropab.h"   // Include ADC library
#include "fdserial.h" 

#define MAX_COMMANDS 20
#define MAX_CMD_LENGTH 128
#define NANO_READY 1
#define RESET_BTN 2

#define M_PI 3.1415926535897932384626433
#define LINK1_LENGTH 60
#define LINK2_LENGTH 60
#define SW 4
#define JOYSTICK_DEADZONE 0.2


//Servo Pins
#define SHOULDER_PIN 15
#define ELBOW_PIN 16
#define WRIST_PIN 17
#define SERVO_ITERATIONS 20 //iterations needed for the servos to be able to reach any angle
#define WRIST_UP 50
#define WRIST_DOWN 70
#define CONSTRAIN(x, lower, upper) ((x < lower) ? lower : ((x > upper) ? upper : x))
#define UPPER_SHOULDER_CONSTRAIN 180
#define LOWER_SHOULDER_CONSTRAIN 0
#define UPPER_ELBOW_CONSTRAIN 180
#define LOWER_ELBOW_CONSTRAIN 0

fdserial *nano_serial = NULL;
static volatile int shoulder_angle = 90;
static volatile int elbow_angle = 90;
static volatile int wrist_angle = WRIST_UP;

static volatile float cur_x = 0;
static volatile float cur_y = LINK1_LENGTH + LINK2_LENGTH;
static volatile float target_x;
static volatile float target_y;

// Function prototypes
void send_string(char str_msg[MAX_CMD_LENGTH]);
void itos(int num, char *str);
void concat(char *dest, char *src);
bool IK(float x, float y, float* theta1, float* theta2);
void parseCoordinates(char* input, int coordinates[MAX_COMMANDS][3], int* count);
void drawJoystickMap(float jx, float jy);
void update_joint_angles(float jx, float jy);
void init_serial(void);
void close_serial(void);
void servo_command(int pin, int angle);
void servo_arms_commands(int shoulder_angle, int elbow_angle);
void all_servos(int commands[3]);

int main() // Main function
{
  pause(1000);
  print("Program Started. Initializing...");
  
  servo_arms_commands(shoulder_angle, elbow_angle);
  servo_command(WRIST_PIN, WRIST_UP);

  char* selection;
  while(1)
  {
    printf("Please choose whether you want to use the pre-programmed drawing - P or use the joystick manually - J: ");
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
                print("joint angles updated: (%d,%d)\n\n", shoulder_angle, elbow_angle);
            }
            else if(get_state(4)){
              if(wrist_angle==WRIST_DOWN){
                 servo_command(WRIST_PIN, WRIST_UP);
                 wrist_angle=WRIST_UP;
              }
              else if(wrist_angle==WRIST_UP){
                 servo_command(WRIST_PIN, WRIST_DOWN);
                 wrist_angle=WRIST_DOWN;
              }
            }               
            pause(500);
            
        }
    }
    else if(!strcmp(selection,"P"))
    {
        char input_str[MAX_CMD_LENGTH] = "d 40,100 40,60 -40,60 -40,100 u";
        //char input_str[] = "u d u d u d";
        //char input_str[] = "u";
        int commands[MAX_COMMANDS][3];
        int count = 0;
        parseCoordinates(input_str, commands, &count);
        printf("num commands = %d\n", count);
        for(int i = 0; i < count; i++)
        {
        printf("command sent : %d, %d, %d\n", commands[i][0], commands[i][1], commands[i][2]);
        all_servos(commands[i]);
        if (get_state(RESET_BTN)) break;
        pause(100);
        }
    }
    else printf("Invalid input. Please enter P or J.\n");
 }

}

void parseCoordinates(char input[MAX_CMD_LENGTH], int coordinates[MAX_COMMANDS][3], int* count) {
    *count = 0;
    char *token = strtok(input, " ");
    float x, y;
    float theta1, theta2;
    char buffer[MAX_CMD_LENGTH]; //Will contain 'angle,angle' so at most 4+1+4 chars
    bool success;

    while(token != NULL && *count < MAX_COMMANDS) {
        printf("token = %s\n",token);
        if(token[0] == 'u') {
            coordinates[*count][0] = -1;
            coordinates[*count][1] = -1;
            coordinates[*count][2] = WRIST_UP;
        }        
        else if(token[0] == 'd') {
            coordinates[*count][0] = -1;
            coordinates[*count][1] = -1;
            coordinates[*count][2] = WRIST_DOWN;
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
                // Restore the comma
                *comma_pos = ',';
            } 
            else {// Invalid format - do nothing
                coordinates[*count][0] = -1;
                coordinates[*count][1] = -1;
                coordinates[*count][2] = -1;
                continue;
            } 
        success = IK(x, y, &theta1, &theta2);

        //Round up theta1 and theta2 and convert to integers
        // theta1 = round(theta1);
        // theta2 = round(theta2);
        
        
        
        if(success)
        {
          // Convert to integers
            int theta1_int = (int)theta1;
            int theta2_int = (int)theta2;  
            coordinates[*count][0] = theta1_int;
            coordinates[*count][1] = theta2_int;
            coordinates[*count][2] = -1;
            shoulder_angle = theta1_int;
            elbow_angle = theta2_int;
            print("angles = %d,%d\n", theta1_int,theta2_int);
        }
        else// IK failed
        {
            coordinates[*count][0] = -1;
            coordinates[*count][1] = -1;
            coordinates[*count][2] = -1;
        }
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

void send_string(char str_msg[MAX_CMD_LENGTH]){
    //init_serial();
    char ndx = -1;
    do {
        ndx++;
        fdserial_txChar(nano_serial, str_msg[ndx]);
    } while (str_msg[ndx] != 0);
    //fdserial_txChar(nano_serial, '\n');
    //close_serial();
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
        if(vx < 0.2) target_x = cur_x - 5;  // Far left: decrement by 2
        else target_x = cur_x - 1;          // Left: decrement by 1
    } else if(vx > 2.5+JOYSTICK_DEADZONE) {
            if(vx > 4.7) target_x = cur_x + 5;  // Far right: increment by 2
        else target_x = cur_x + 1;          // Right: increment by 1
    }

    // Y-axis controls Y-position
    if(vy < 2.5-JOYSTICK_DEADZONE) {
        if(vy < 0.2) target_y = cur_y - 5;     // Far down: decrement by 2
        else target_y = cur_y - 1;             // Down: decrement by 1
    } else if(vy > 2.5+JOYSTICK_DEADZONE) {
        if(vy > 4.7) target_y = cur_y + 5;     // Far up: increment by 2
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
        if(target_shoulder_angle < 0) target_shoulder_angle = 0;
        if(target_shoulder_angle > 180) target_shoulder_angle = 180;
        
        if(target_elbow_angle < 0) target_elbow_angle = 0;
        if(target_elbow_angle > 180) target_elbow_angle = 180;

        // char updated_angles_str[MAX_CMD_LENGTH];
        target_shoulder_angle = (int)target_shoulder_angle;
        target_elbow_angle = (int)target_elbow_angle;

        if (target_shoulder_angle == shoulder_angle && target_elbow_angle == elbow_angle) return;
        else if (target_shoulder_angle == shoulder_angle && target_elbow_angle != elbow_angle)
        {
            servo_arms_commands(-1, target_elbow_angle);
            elbow_angle = target_elbow_angle;
        }
        else if (target_shoulder_angle != shoulder_angle && target_elbow_angle == elbow_angle)
        {
            servo_arms_commands(target_shoulder_angle, -1);
            shoulder_angle = target_shoulder_angle;
        }
        else
        {
        servo_arms_commands(target_shoulder_angle, target_elbow_angle);
        shoulder_angle = target_shoulder_angle;
        elbow_angle = target_elbow_angle;
        }
        print("Target point %f, %f reached with angles %f, %f\n", target_x, target_y, target_shoulder_angle, target_elbow_angle);
    }

    
}

void init_serial() {
    if (nano_serial == NULL) {
        nano_serial = fdserial_open(1, 0, 0, 9600);
    }
}

void close_serial() {
    if (nano_serial != NULL) {
        fdserial_close(nano_serial);
        nano_serial = NULL;
    }
}


void servo_command(int pin, int angle){
  for (int i = 0; i < SERVO_ITERATIONS ; i++){
    pulse_out(pin, (500+10*angle));
    pause(20);
    }
}

void servo_arms_commands(int shoulder_angle, int elbow_angle)
{
    if(shoulder_angle!= -1) servo_command(SHOULDER_PIN, CONSTRAIN(shoulder_angle, LOWER_SHOULDER_CONSTRAIN, UPPER_SHOULDER_CONSTRAIN));
    if(elbow_angle!= -1) servo_command(ELBOW_PIN, 180 - CONSTRAIN(elbow_angle, LOWER_ELBOW_CONSTRAIN, UPPER_ELBOW_CONSTRAIN));
}

void all_servos(int commands[3])
{
    if(commands[0] == -1 && commands[1] == -1 && commands[2] == -1) return;
    servo_arms_commands(commands[0], commands[1]);
    if(commands[2] != -1) servo_command(WRIST_PIN, commands[2]);
}

