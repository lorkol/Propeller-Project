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
#define SERVO_ITERATIONS 20 //iterations needed for the servos to be able to reach any angle
#define WRIST_UP 40
#define WRIST_DOWN 90

// Function prototypes
void send_string(char str_msg[MAX_CMD_LENGTH]);
void itos(int num, char *str);
void concat(char *dest, char *src);
void input_string();
bool IK(float x, float y, float* theta1, float* theta2);
void parseCoordinates(char* input, char coordinates[MAX_COMMANDS][MAX_CMD_LENGTH], int* count);
void drawJoystickMap(float jx, float jy);
void update_joint_angles(float jx, float jy);
void init_serial(void);
void close_serial(void);
void servo_command(int pin, int angle);

int main() // Main function
{
  pause(1000);
  print("Program Started. Initializing...");
  init_serial();
  
  char updated_angles_str[MAX_CMD_LENGTH];
  sprintf(updated_angles_str, "%d,%d", shoulder_angle, elbow_angle);
  while(!get_state(NANO_READY)){}
  send_string(updated_angles_str); 
  while(!get_state(NANO_READY)){}
  send_string("u");

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
            
            if (get_state(NANO_READY) && (lrV < (2.5-JOYSTICK_DEADZONE) || udV < (2.5-JOYSTICK_DEADZONE) || lrV > (2.5+JOYSTICK_DEADZONE) || udV > (2.5+JOYSTICK_DEADZONE))){
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
        char commands[MAX_COMMANDS][MAX_CMD_LENGTH];
        int count = 0;
        parseCoordinates(input_str, commands, &count);
        printf("num commands = %d\n", count);
        for(int i = 0; i < count; i++)
        {
        printf("command sent : %s\n", commands[i]);
        while(!get_state(NANO_READY)){}
        if(strcmp(commands[i], "")) send_string(commands[i]); //Send the command if it's not empty
        if (get_state(RESET_BTN)) break;
        pause(100);
        }
    }
    else printf("Invalid input. Please enter P or J.\n");
 }

  close_serial();
}

void parseCoordinates(char input[MAX_CMD_LENGTH], char coordinates[MAX_COMMANDS][MAX_CMD_LENGTH], int* count) {
    *count = 0;
    char *token = strtok(input, " ");
    float x, y;
    float theta1, theta2;
    char buffer[MAX_CMD_LENGTH]; //Will contain 'angle,angle' so at most 4+1+4 chars
    bool success;

    while(token != NULL && *count < MAX_COMMANDS) {
      printf("token = %s\n",token);
      if(token[0] == 'u') strcpy(coordinates[*count],"u");
      else if(token[0] == 'd') strcpy(coordinates[*count],"d");
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
        } else {
          strcpy(coordinates[*count], ""); // Invalid format
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
          sprintf(coordinates[*count], "%d,%d", theta1_int, theta2_int);
          print("angles = %d,%d\n", theta1_int,theta2_int);
        }
        else strcpy(coordinates[*count], ""); // IK failed
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

void input_string() {
    int shoulder_angle, elbow_angle, wrist_angle;
    
    print("Enter joint angles: ");
    scanf("%d %d %d\n", &shoulder_angle, &elbow_angle, &wrist_angle);

    char strA[3];
    char strB[3];
    char strC[3];

    itos(shoulder_angle, strA);
    itos(elbow_angle, strB);
    itos(wrist_angle, strC);

    char result[12];
    result[0] = '\0'; // Initialize result as an empty string

    concat(result, strA);
    concat(result, " ");
    concat(result, strB);
    concat(result, " ");
    concat(result, strC);
    concat(result, "\n");
    
    send_string(result);
    printf("%s", result);
    return;
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
        sprintf(updated_angles_str, "%d,%d", -1, new_elbow_angle);
        send_string(updated_angles_str);
    }
    else if (new_shoulder_angle != shoulder_angle && new_elbow_angle == elbow_angle)
    {
        sprintf(updated_angles_str, "%d,%d", new_shoulder_angle, -1);
        send_string(updated_angles_str);
    }
    else
    {
      sprintf(updated_angles_str, "%d,%d", new_shoulder_angle, new_elbow_angle);
      send_string(updated_angles_str);
    }

    shoulder_angle = new_shoulder_angle;
    elbow_angle = new_elbow_angle;
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
    }
}