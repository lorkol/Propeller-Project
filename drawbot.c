#include "simpletools.h"                      // Include simple tools
#include "adcDCpropab.h"   // Include ADC library
#include "fdserial.h" 
#include "Queues.h"
#include "Utils.h"

#define MAX_COMMANDS 20
#define MAX_CMD_LENGTH 9
#define NANO_READY 1
#define RESET_BTN 2

#define SW 4

fdserial *nano_serial = NULL;
static volatile int elbow_angle = 90;
static volatile int shoulder_angle = 90;
static volatile int wrist_angle = 40;
static volatile StringQueue msg_queue;
unsigned int stack_messages[40 + 25];

// Function prototypes
void send_string(char str_msg[MAX_CMD_LENGTH]);
void input_string();
void parseCoordinates(char* input, char coordinates[MAX_COMMANDS][MAX_CMD_LENGTH], int* count);
void drawJoystickMap(float jx, float jy);
void init_serial();
void close_serial();
void messages_cog();

int main() // Main function
{
  init_serial();
  initQueue(&msg_queue);
  cogstart(&messages_cog, NULL, stack_messages, sizeof(stack_messages));
  pause(1000);

  char updated_angles_str[MAX_CMD_LENGTH];
  sprintf(updated_angles_str, "%d,%d", shoulder_angle, elbow_angle);
  enqueue(&msg_queue, updated_angles_str); //MIGHT NEED TO MODIFY THIS INPUT FORMAT OR MODIFY THE FUNCTION INPUT FORMAT
  enqueue(&msg_queue, "u");

  char selection;
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
                
                
            drawJoystickMap(lrV, udV);  // Display joystick movement as a map
            
            if (get_state(NANO_READY)) update_joint_angles(lrV, udV);  // Update joint angles

            pause(100);
            if (get_state(RESET_BTN)) break;  // Update every 100ms
        }
    }
    else if(!strcmp(selection,"P"))
    {
        char input_str[] = "1,2 d 6,5 3,3 u 4,5 1,1";
        //char input_str[] = "u";
        char commands[MAX_COMMANDS][MAX_CMD_LENGTH];
        int count = 0;
        pause(2000);
        parseCoordinates(input_str, commands, &count);
        printf("num commands = %d\n", count);
        for(int i = 0; i < count; i++)
        {
        printf("command = %s\n", commands[i]);
        if(strcmp(commands[i], "")) enqueue(&msg_queue, commands[i]); //Send the command if it's not empty
        if (get_state(RESET_BTN)) break;
        }
    }
    else printf("Invalid input. Please enter P or J.\n");
  }

  close_serial();
  //TODO stop the cogs
  
}

void send_string(char str_msg[MAX_CMD_LENGTH]) {
    char ndx = -1;
    do {
        ndx++;
        fdserial_txChar(nano_serial, str_msg[ndx]);
    } while (str_msg[ndx] != 0);
    fdserial_txChar(nano_serial, '\n');
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
    
    enqueue(&msg_queue, result);
    printf("%s", result);
    return;
}

void parseCoordinates(char* input, char coordinates[MAX_COMMANDS][MAX_CMD_LENGTH], int* count) {
    *count = 0;
    
    char *token = strtok(input, " ");
    float x, y;
    int theta1, theta2;
    char buffer[MAX_CMD_LENGTH]; //Will contain 'angle,angle' so at most 4+1+4 chars
    bool success;
    
    pause(1500);
    while(token != NULL && *count < MAX_COMMANDS) {
      printf("token = %s\n",token);
      if(token[0] == 'u') strcpy(coordinates[*count],"u");
      else if(token[0] == 'd') strcpy(coordinates[*count],"d");
      else 
      {
        sscanf(token, "%f,%f", &x, &y);
        success = IK(x, y, &theta1, &theta2);
        if(success)
        {
          sprintf(coordinates[*count], "%d,%d", theta1, theta2);
          printf("angles = %d,%d\n", theta1,theta2);
        }
        else strcpy(coordinates[*count], ""); // IK failed
      }

      (*count)++;
      token = strtok(NULL, " ");  // Get the next token
    }
}

void drawJoystickMap(float jx, float jy) {
    char grid[5][5] = {
        {'-', '-', '-', '-', '-'},
        {'-', '-', '-', '-', '-'},
        {'-', '-', 'O', '-', '-'},  // Default center position
        {'-', '-', '-', '-', '-'},
        {'-', '-', '-', '-', '-'}
        };

    // Convert voltage (0V-4V) to grid coordinates (0-4)
    int gridx = (int)(jx);
    int gridy = (int)(jy);

    // Ensure values stay within 0-4 range
    if (gridx > 4) gridx = 4;
    if (gridy > 4) gridy = 4;

    // Invert Y-axis so UP is at the top of the grid
    gridy = 4 - gridy;

    // Place joystick position on the grid
    grid[gridy][gridx] = 'X';

    
    print("%c",HOME);  // Clear screen and move cursor to top-left

    print("\nJoystick Position:\n");

    // Print the 5x5 grid
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
            print("%c ", grid[i][j]);
        }
        print("\n");
    }
}

void update_joint_angles(float jx, float jy) {
    // Convert voltage (0V-4V) to grid coordinates (0-4)
    int xStep = (int)(jx);
    int yStep = (int)(jy);

    // Ensure values stay within 0-4 range
    if (xStep > 4) xStep = 4;
    if (yStep > 4) yStep = 4;

    // Invert Y-axis so UP is at the top of the grid
    yStep = 4 - yStep;

    // Update joint angles based on joystick position. If xStep/yStep is 2 then no change. 
    // If xStep/yStep is 1 decrement shoulder_angle/elbow_angle by 1 degree.
    // If xStep/yStep is 0 decrement shoulder_angle/elbow_angle by 2 degrees.  

    int new_shoulder_angle = shoulder_angle;
    int new_elbow_angle = elbow_angle;
    
    if(xStep == 1) new_shoulder_angle--;
    if(xStep == 0) new_shoulder_angle -= 2;
    if(xStep == 3) new_shoulder_angle++;
    if(xStep == 4) new_shoulder_angle += 2;
    
    if(yStep == 1) new_elbow_angle--;
    if(yStep == 0) new_elbow_angle -= 2;
    if(yStep == 3) new_elbow_angle++;
    if(yStep == 4) new_elbow_angle += 2;

    char updated_angles_str[MAX_CMD_LENGTH];
    sprintf(updated_angles_str, "%d,%d", new_shoulder_angle, new_elbow_angle);
    enqueue(&msg_queue, updated_angles_str); //MIGHT NEED TO MODIFY THIS INPUT FORMAT OR MODIFY THE FUNCTION INPUT FORMAT
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

void messages_cog()
{
    while(1)
    {
    while(!get_state(NANO_READY)){}
    send_string(dequeue(&msg_queue)); //Send the command if it's not empty
    }
}