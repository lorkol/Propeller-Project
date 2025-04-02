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

fdserial *nano_serial = NULL;
static volatile int elbow_angle = 90;
static volatile int shoulder_angle = 90;
static volatile int wrist_angle = 40;

//Servo Pins
#define SHOULDER_PIN 15
#define ELBOW_PIN 16
#define WRIST_PIN 17

//Servo constants
#define SERVO_ITERATIONS 20 //iterations needed for the servos to be able to reach any angle
#define WRIST_UP 40
#define WRIST_DOWN 90
#define CONSTRAIN(x, lower, upper) ((x < lower) ? lower : ((x > upper) ? upper : x))
#define UPPER_SHOULDER_CONSTRAIN 180
#define LOWER_SHOULDER_CONSTRAIN 0
#define UPPER_ELBOW_CONSTRAIN 180
#define LOWER_ELBOW_CONSTRAIN 0

struct Command{
    int shoulder_command;
    int elbow_command;
    int wrist_command;
};

// Function prototypes
void itos(int num, char *str);
void concat(char *dest, char *src);
bool IK(float x, float y, float* theta1, float* theta2);
void parseCoordinates(char* input, Command coordinates[MAX_COMMANDS], int* count);
void drawJoystickMap(float jx, float jy);
void update_joint_angles(float jx, float jy);
void servo_command(int pin, int angle);
void servo_arms_commands(int shoulder_angle, int elbow_angle);
void all_servos(Command command);
void init_servo_cogs();
void close_cogs();

void shoulder_movement(void *par);
void elbow_movement(void *par);
void wrist_movement(void *par);

static volatile unsigned int servos_working = 0;
static volatile Command current_command = {-1, -1, -1};
static volatile bool shoulder_working = false;
static volatile bool elbow_working = false;
static volatile bool wrist_working = false;
unsigned int shoulder_stack[40+25];
unsigned int elbow_stack[40+25];
unsigned int wrist_stack[40+25];

int main() // Main function
{
  pause(1000);
  print("Program Started. Initializing...");
  init_servo_cogs();
  all_servos({shoulder_angle, elbow_angle, WRIST_UP});

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
            
            if ((lrV < (2.5-JOYSTICK_DEADZONE) || udV < (2.5-JOYSTICK_DEADZONE) || lrV > (2.5+JOYSTICK_DEADZONE) || udV > (2.5+JOYSTICK_DEADZONE))){
                update_joint_angles(lrV, udV);  // Update joint angles
                print("lrV = %f, udV = %f\n", lrV, udV);
                print("joint angles updated: (%d,%d)\n", shoulder_angle, elbow_angle);
            } 

            //pause(100);
        }
    }
    else if(!strcmp(selection,"P"))
    {
        char input_str[MAX_CMD_LENGTH] = "60,60 60,61 60,62 60,63 60,64 60,68 60,69 60,70 60,72 60,75 60,78 60,80";
        //char input_str[] = "u d u d u d";
        //char input_str[] = "u";
        Command commands[MAX_COMMANDS];
        int count = 0;
        parseCoordinates(input_str, commands, &count);
        printf("num commands = %d\n", count);
        for(int i = 0; i < count; i++)
        {
            printf("command sent : %d, %d, %d\n", commands[i].shoulder_command, commands[i].elbow_command, commands[i].wrist_command);
            all_servos(commands[i]);
            if (get_state(RESET_BTN)) break;
            pause(100);
        }
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
        if(token[0] == 'u') coordinates[*count] = {-1, -1, WRIST_UP};
        else if(token[0] == 'd') coordinates[*count] = {-1, -1, WRIST_DOWN};
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
                coordinates[*count] = {-1, -1, -1};
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
            coordinates[*count] = {theta1_int, theta2_int, -1};
            print("angles = %d,%d\n", theta1_int,theta2_int);
        }
        else// IK failed
            coordinates[*count] = {-1, -1, -1};
      }

      (*count)++;
      token = strtok(NULL, " ");  // Get the next token
    }
}



bool IK(float x, float y, float* theta1, float* theta2) {
    float c2 = (x*x + y*y - LINK1_LENGTH*LINK1_LENGTH - LINK2_LENGTH*LINK2_LENGTH)/(2*LINK1_LENGTH*LINK2_LENGTH);

    // Print input coordinates first for debugging
    print("IK for point: x=%f, y=%f\n", x, y);
          
    print("c2 = %d.%d\n", (int)c2, (int)(c2*100)%100);

    if(c2 >= -1 && c2 <= 1 && y >= 0)
    {
        // float s2 = sqrt(1 - c2*c2);  // Positive solution TODO add negative solution when optimizing
        
        // float t1 = atan2(-LINK2_LENGTH*s2*x + (LINK1_LENGTH+LINK2_LENGTH*c2)*y,
        //                 (LINK1_LENGTH + LINK2_LENGTH*c2)*x + LINK2_LENGTH*s2*y);
        
        float alpha = acos((x*x + y*y + LINK1_LENGTH*LINK1_LENGTH - LINK2_LENGTH*LINK2_LENGTH)/(2*LINK1_LENGTH*sqrt(x*x + y*y)));
        float beta = acos((-x*x - y*y + LINK1_LENGTH*LINK1_LENGTH + LINK2_LENGTH*LINK2_LENGTH)/(2*LINK1_LENGTH*LINK2_LENGTH));
        float gamma = atan2(y,x);
        
        // Convert to integer degrees multiplied by 100 for printing
        print("alpha=%f, beta=%f, gamma=%f\n", alpha, beta, gamma);
        
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
        shoulder_angle = (int)*theta1;
        elbow_angle = (int)*theta2;
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

    // Ensure values stay within 0-4 range
    if (xStep > 4) xStep = 4;
    if (yStep > 4) yStep = 4;

    // // Invert Y-axis so UP is at the top of the grid
    // yStep = 4 - yStep;

    // Update joint angles based on joystick position. If xStep/yStep is 2 then no change. 
    // If xStep/yStep is 1 decrement shoulder_angle/elbow_angle by 1 degree.
    // If xStep/yStep is 0 decrement shoulder_angle/elbow_angle by 2 degrees.  

    int new_shoulder_angle = shoulder_angle;
    int new_elbow_angle = elbow_angle;
    
    
    
    // Constrain angles between 0 and 180 degrees
    if(new_shoulder_angle < 0) new_shoulder_angle = 0;
    if(new_shoulder_angle > 180) new_shoulder_angle = 180;
    
    if(new_elbow_angle < 0) new_elbow_angle = 0;
    if(new_elbow_angle > 180) new_elbow_angle = 180;

    char updated_angles_str[MAX_CMD_LENGTH];

    if (new_shoulder_angle == shoulder_angle && new_elbow_angle == elbow_angle) return;
    else if (new_shoulder_angle == shoulder_angle && new_elbow_angle != elbow_angle)
    {
        servo_arms_commands(-1, new_elbow_angle);
    }
    else if (new_shoulder_angle != shoulder_angle && new_elbow_angle == elbow_angle)
    {
        servo_arms_commands(new_shoulder_angle, -1);
    }
    else
      servo_arms_commands(new_shoulder_angle, new_elbow_angle);

    shoulder_angle = new_shoulder_angle;
    elbow_angle = new_elbow_angle;
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
     || (command.shoulder_command == -1 && command.elbow_command == -1 && command.wrist_command == -1)) return;
    while(wrist_working || shoulder_working || elbow_working){};
    current_command.shoulder_command = command.shoulder_command;
    current_command.elbow_command = command.elbow_command;
    current_command.wrist_command = command.wrist_command;
    if(command.shoulder_command != -1) shoulder_working = true;
    if(command.elbow_command != -1) elbow_working = true;
    if(command.wrist_command != -1) wrist_working = true;
    
}

void servo_arm_commands(int shoulder_angle, int elbow_angle)
{
    if(current_command.shoulder_command == -1 && current_command.elbow_command == -1) return;
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
        wrist_working = false;
    }
}


void servo_command(int pin, int angle){
  for (int i = 0; i < SERVO_ITERATIONS ; i++){
    pulse_out(pin, (500+10*angle));
    pause(20);
    }
}
