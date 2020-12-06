#include <stdlib.h>
#include <stdio.h>
#include <string.h>

int main() {
    // create a character array (i.e. string) to hold the values from fscanf()
    char thisString[50];
    int value;
    int a;
    int agent_locations[3][4];

    // create a pointer to the file that will be opened
    FILE *filePointer = NULL;

    // assign the pointer to the result of opening the file with fopen()
    filePointer = fopen("./GameMaps/test6.loc2", "r+");

    if (filePointer != NULL) {
        while (fscanf(filePointer, "%s", thisString) == 1) {
            if (fscanf(filePointer, "%d", &value) == 1) {
                if (strcmp(thisString, "a") == 0)
                    a = value;
                else if (strcmp(thisString, "goaly") == 0)
                    agent_locations[a][0] = value;
                else if (strcmp(thisString, "goalx") == 0)
                    agent_locations[a][1] = value;
                else if (strcmp(thisString, "starty") == 0)
                    agent_locations[a][2] = value;
                else if (strcmp(thisString, "startx") == 0)
                    agent_locations[a][3] = value;
            }
        }
    }

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            printf("%d - ", agent_locations[i][j]);
        }
        printf("\n");
    }
    

    fclose(filePointer);
    filePointer = NULL;
}