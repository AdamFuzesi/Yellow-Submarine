/*
* 

Author: Adam C. Fuzesi

Language: C

Framework and libraries: OpenGL, freeglut

Online Sources and Articles used:
https://www.opengl-tutorial.org/beginners-tutorials/tutorial-7-model-loading/
https://www.manytextures.com/category/sand/
https://en.wikipedia.org/wiki/Boids
https://howthingsfly.si.edu/flight-dynamics/roll-pitch-and-yaw#:~:text=Rotation%20around%20the%20side%2Dto,vertical%20axis%20is%20called%20yaw.
https://en.wikipedia.org/wiki/Netpbm


*/
#define _CRT_SECURE_NO_WARNINGS 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define _USE_MATH_DEFINES 
#include <math.h>
#include <glut.h>
#include <ctype.h> 



// for the boids implementation
#define NUMBER_OF_BOIDS 80
#define NEIGHBORS 3
#define DISTANCE 10.0f
#define SEPARATION 3.0f
#define WALL_YIELD_STEER 8.0f
#define MAX_SPEED 0.2f
#define MIN_SPEED 0.05f


int windowWidth = 800;
int windowHeight = 600;
int windowPosX = 100;
int windowPosY = 100;
// setting initial window but fullscreen is so much better
int wireframeMode = 0;  
// flag 1 activate the wireframe
int fullscreenMode = 0;  
// setting flag states for each window mode

float cameraDistance = 30.0f;
// change if needed for larger view
float xAngle = 0.0f;
float yAngle = 0.0f;
// sets good third person perspectivv

// sub position/orientation
float submarineYaw = 0.0f; 
// orientation logic for sub
float submarinePosX = 0.0f;
float submarinePosY = 0.0f;
float submarinePosZ = 0.0f;

int moveForward = 0;
int moveBackward = 0;
int moveLeft = 0;
int moveRight = 0;
int moveUp = 0;
int moveDown = 0;
// used to set states for my movement logic
// Mouse movement variables
int prevMouseX = -1;
int prevMouseY = -1;
GLuint cylinderTextureID = 0;
float cylinderHeight = 45.0f;

// same logic as the 2d program for the boids to simulate actual life 
typedef struct {
    float position[3];
    float velocity[3];
} Boid;

Boid* CurFlock = NULL;
Boid* PrevFlock = NULL;
// same logic from assingnment 1 
void initializeBoids();
void boidLogic();
void drawoutBoids();

typedef struct {
    float x;
    float y;
    float z;
} Vertex;

typedef struct {
    float x;
    float y;
    float z;
} Normal;

typedef struct {
    int v[3];
    // vertex indices 
    int n[3];
    // normals
} Face;

Vertex* vertices = NULL;
Normal* normals = NULL;
Face* faces = NULL;
int numOfVertices = 0;
int numOfNormals = 0;
int numOfFaces = 0;
// sub metrics for load in

// logic for the coral implementations

typedef struct {
    float x;
    float y;
    float z;
} CoralNormal;

typedef struct {
    float x;
    float y;
    float z;
} CoralVerts;

typedef struct {
    int v[3];
    int n[3]; 
    // same proces logic as sub model
} CoralFace;

typedef struct {
    CoralVerts* vertices;
    CoralNormal* normals;
    CoralFace* faces;
    int numVertices;
    int numNormals;
    int numFaces;
} CoralStructure;
// contained this in a struc instead, was initially going to do the same thing as my sub but since its loading in multiple... made more sense

typedef struct {
    CoralStructure* model;  
    float position[3];
    float rotation[3]; 
    float scale;
} CoralInstances;

// storiong in arrays to keep track of them easier... was going to randomize generation but proved to be a bad way fo doing it
#define NUM_CORAL_MODELS 14
#define NUM_CORALS 35
CoralStructure coralModels[NUM_CORAL_MODELS];
CoralInstances corals[NUM_CORALS];
void coralModelLoad(const char* filename, CoralStructure* model);
void initializeCorals();
void coralDrawout(CoralStructure* model);
void drawCorals();

GLuint baseTextID = 0;

typedef struct {
    unsigned char* data;
    int width;
    int height;
} PPMIn;
// struct to cater to PPM load in metrics

int fogEnabled = 1; 
// flag for fog 
GLUquadric* quadricDisk = NULL;
GLUquadric* quadricCylinder = NULL;
// setting quadric view
int waterGridSize = 100;
float waterPlaneSize = 200.0f;
float* waterVertices = NULL; 
// holds recursive vertexes trhough wave motion
unsigned int* waterIndices = NULL;
// metrics for water surface

// Function prototypes
void init();
void shapeReset(int width, int height);
void display();
void movementLogic();
void keyboardFunc(unsigned char key, int x, int y);
void keyboardUpFunc(unsigned char key, int x, int y);
void specialKeys(int key, int x, int y);
void specialKeysUp(int key, int x, int y);
void passiveMotionFunc(int x, int y);
void loadSubmarineModel(const char* filename);
void drawoutSub();
void centerAxis();
void drawSeaFloor();
void drawWallEnv();
void initWater();
void updateWater();
void drawWater();
void printKeyboardControls();
GLuint loadTexture(const char* filename);
PPMIn* loadPPM(const char* filename);
void cleanup();

int main(int argc, char** argv) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(windowWidth, windowHeight);
    glutInitWindowPosition(windowPosX, windowPosY);
    // setting up window and program frame
    glutCreateWindow("Final Project CSCI3161");

    init();
    glutDisplayFunc(display);
    glutReshapeFunc(shapeReset);
    glutIdleFunc(movementLogic);
    glutKeyboardFunc(keyboardFunc);
    glutKeyboardUpFunc(keyboardUpFunc);
    // setting callback functions
    glutSpecialFunc(specialKeys);
    glutSpecialUpFunc(specialKeysUp);
    glutPassiveMotionFunc(passiveMotionFunc);
    initializeBoids();
    printKeyboardControls();

    glutMainLoop();

    cleanup();

    return 0;
}
// initalizing the program, also sets up main env
void init() {
    glClearColor(0.0, 0.0, 0.4, 1.0);
    // matches well with waves
    glEnable(GL_DEPTH_TEST); 
    glEnable(GL_LIGHTING);   
    glEnable(GL_LIGHT0);   
    glEnable(GL_NORMALIZE);       
    glShadeModel(GL_SMOOTH); 

    // Set up light properties
    GLfloat lightAmbient[] = {  0.2f, 0.2f, 0.2f, 1.0f };
    GLfloat lightDiffuse[] = { 0.8f, 0.8f, 0.8f, 1.0f };
    GLfloat lightSpecular[] = { 1.0f, 1.0f, 1.0f, 1.0f  };
    GLfloat lightPosition[] = { 0.0f, 10.0f, 0.0f, 0.0f };
    // using directional light properties
    glLightfv(GL_LIGHT0, GL_AMBIENT, lightAmbient );
    glLightfv(GL_LIGHT0, GL_DIFFUSE, lightDiffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, lightSpecular);
    glLightfv( GL_LIGHT0, GL_POSITION, lightPosition);
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);

    quadricDisk = gluNewQuadric();
    // quadrics view
    gluQuadricTexture(quadricDisk, GL_TRUE); 
    // textur mapping/setting for the bottom layer
    gluQuadricNormals(quadricDisk, GLU_SMOOTH);
    quadricCylinder = gluNewQuadric();
    gluQuadricTexture(quadricCylinder, GL_TRUE);
    gluQuadricNormals(quadricCylinder, GLU_SMOOTH);
    loadSubmarineModel("submarine - updated.obj");
    baseTextID = loadTexture("sand_ascii.ppm");
    cylinderTextureID = loadTexture("smoothSandpt2.ppm");
    // loading in all my files to implement as texture towards environment models
    GLfloat fogColor[] = { 0.1f, 0.1f, 0.4f, 1.0f }; 
    glFogi(GL_FOG_MODE, GL_EXP);
    glFogfv(GL_FOG_COLOR, fogColor );
    glFogf(GL_FOG_DENSITY, 0.01f);
    // fog params, assingmnet pics looks closely to the 0.01f density setting

    initializeCorals();
    initWater();
    // recursive water surface waves
}

void initWater() {
    // initializes water sruface
    int numberOfVertices = (waterGridSize + 1) * (waterGridSize + 1);
    int numberOfIndices = waterGridSize * waterGridSize * 6;

    waterVertices = (float*)malloc(sizeof(float) * numberOfVertices * 3); // x, y, z per vertex
    waterIndices = (unsigned int*)malloc(sizeof(unsigned int) * numberOfIndices);

    int index = 0;
    // making vertex positions, will be used in wave algo to create wave feature
    for (int i = 0; i <= waterGridSize; ++i) {
        for (int j = 0; j <= waterGridSize; ++j) {
            float x = ((float)j / waterGridSize - 0.5f) * waterPlaneSize;
            float y = 0.0f;
            // initial height
            float z = ((float)i / waterGridSize - 0.5f) * waterPlaneSize;
            waterVertices[index++] = x;
            waterVertices[index++] = y;
            waterVertices[index++] = z;
        }
    }

    index = 0;
    // indices
    for (int i = 0; i < waterGridSize; ++i) {
        for (int j = 0; j < waterGridSize; ++j) {
            int row1 = i * (waterGridSize + 1);
            int row2 = (i + 1) * (waterGridSize + 1);
            waterIndices[index++] = row1 + j;
            waterIndices[index++] = row2 + j;
            waterIndices[index++] = row1 + j + 1;
            waterIndices[index++] = row1 + j + 1;
            waterIndices[index++] = row2 + j;
            waterIndices[index++] = row2 + j + 1;
            // setting triangle indices
        }
    }
}

void centerAxis() {
    // fucntions draws out coordinate axis initially (stays there)
    glLineWidth(6.0f);
    glBegin(GL_LINES);
    glColor3f(1.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(5.0f, 0.0f, 0.0f);
    glColor3f(0.0f, 1.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 5.0f, 0.0f);
    glColor3f(0.0f, 0.0f, 1.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 5.0f);
    glEnd();
}

void shapeReset(int width, int height) {
    glViewport(0, 0, width, height);
    windowWidth = width;
    windowHeight = height;

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0, (float)width / (float)height, 1.0, 1000.0);

    glMatrixMode(GL_MODELVIEW);
}

void display() {
    // handles displaying all params, identities and objects setting up the environment itself
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

    if (wireframeMode) {
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glColor3f(1.0f, 1.5f, 0.0f);
        // wireframe yellow

    }
    else {
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        // setting default val; for wireframe activiation
        glColor3f(1.0f, 1.0f, 1.0f); 
    }
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    float cameraXPos = submarinePosX + cameraDistance * sinf(yAngle) * cosf(xAngle );
    float cameraYPos = submarinePosY + cameraDistance * sinf(xAngle);
    float cameraZPos = submarinePosZ + cameraDistance * cosf(yAngle) * cosf(xAngle);

    gluLookAt(cameraXPos, cameraYPos, cameraZPos, submarinePosX, submarinePosY, submarinePosZ, 0.0, 1.0, 0.0 ); 
    // sets up the cameras perspective
    GLfloat directionalLighting[] = { 0.0f, 10.0f, 0.0f, 0.0f };
    // pos to cam
    // directionall light to assets
    glLightfv(GL_LIGHT0, GL_POSITION, directionalLighting );

    if (fogEnabled) {

        glEnable(GL_FOG);
    }

    else {

        glDisable(GL_FOG);
    }
    // fog enabling

    drawSeaFloor();
    drawWallEnv();
    // draws out the environmnent 
    centerAxis();
    // axis orientation 

    GLUquadric* quad = gluNewQuadric();
    gluSphere(quad, 0.5f, 20, 20);
    gluDeleteQuadric(quad);

    drawoutSub();
    drawoutBoids();
    drawCorals();
    // drawing out objects/models
    drawWater();

    glutSwapBuffers();
}

// Idle function
void movementLogic() {
    float speed = 0.2f;

    if (moveForward) {
        submarinePosX += speed * sinf(submarineYaw * M_PI / 180.0f);
        submarinePosZ += speed * cosf(submarineYaw * M_PI / 180.0f);
    }
    if (moveBackward) {
        submarinePosX -= speed * sinf(submarineYaw * M_PI / 180.0f);
        submarinePosZ -= speed * cosf(submarineYaw * M_PI / 180.0f);
    }
    if (moveLeft) {
        submarineYaw += 2.0f;
        // left rotation
    }
    if (moveRight) {
        submarineYaw -= 2.0f;
        // right rotation
    }
    if (moveUp) {
        submarinePosY += speed;
        // moves upwards
    }
    if (moveDown) {
        submarinePosY -= speed; 
        // moves downwards
    }

    boidLogic();
    updateWater();

    glutPostRedisplay();
}

void updateWater() {
    // handles the animation to recursively output a wave effect
    float time = glutGet(GLUT_ELAPSED_TIME) / 1000.0f;
    float waveAmplitude = 1.5f;
    // controls the waves heights
    float waveFrequency = 0.5f;
    
    // itervals for wave effect

    for (int i = 0; i <= waterGridSize; ++i) {
        for (int j = 0; j <= waterGridSize; ++j) {

            int index = (i * (waterGridSize + 1 ) + j) * 3;
            float x = waterVertices[index];
            float z = waterVertices[index + 2];

            // calculates height using specifications sine functions
            // different sort of waves as specs, made smaller patterns to create more of an ocean look to it 
            float height = sinf(waveFrequency * (x + time)) * waveAmplitude+ sinf(waveFrequency * (z + time * 0.5f)) * (waveAmplitude * 0.5f);

            waterVertices[index + 1] = height;
        }
    }
}

// ffunction to draw the water surface
void drawWater() {
    glPushMatrix();

    // getting the waters surface Y-position
    float cylinderBaseY = -2.5f;
    //  base position vertically
    float waterYPosition = cylinderBaseY + cylinderHeight + 0.1f;
    // looks good at this height, makes waves more aparent

    // Move water surface to desired height
    glTranslatef(0.0f, waterYPosition, 0.0f);

    // Set water color and material
    if (wireframeMode) {
        glColor3f(0.0f, 0.5f, 1.0f);
        // nice light blue color for wireframe makes it easy to visualize
    }

    else {
        glDisable(GL_COLOR_MATERIAL); 
        // when i lave it still wihtout disable it turns into
        GLfloat waterAmbient[] = { 0.0f, 0.2f, 0.4f, 0.7f };

        GLfloat waterDiffuse[] = { 0.0f, 0.3f, 0.6f, 0.7f };

        GLfloat waterSpecular[] = { 0.8f, 0.8f, 0.8f, 0.7f };

        GLfloat waterShininess = 50.0f;
   

        glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, waterAmbient);
        glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, waterDiffuse);
        glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, waterSpecular);
        glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, waterShininess);
        // activating set lighting conditions

        glColor4f(0.0f, 0.3f, 0.6f, 0.7f);
        // blending state to make it look sync

        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    }

    glEnableClientState(GL_VERTEX_ARRAY);
    // drawing out the surf
    glVertexPointer(3, GL_FLOAT, 0, waterVertices );
    glDrawElements( GL_TRIANGLES, waterGridSize * waterGridSize * 6, GL_UNSIGNED_INT, waterIndices);
    glDisableClientState(GL_VERTEX_ARRAY);

    if (!wireframeMode) {
        glDisableClientState(GL_TEXTURE_COORD_ARRAY);
        glDisable(GL_TEXTURE_2D);
        glDisable(GL_BLEND);
        glEnable(GL_COLOR_MATERIAL);
        // setting back to solid state mode 
    }

    glPopMatrix();
}


// functions identifying and setting program controls
void keyboardFunc(unsigned char key, int x, int y) {
    // used the same principle logic as the last 2 assignments
    switch (key) {

    case 'u':
    case 'U':
        wireframeMode = !wireframeMode;
        break;

    case 'f':
    case 'F':
        if (fullscreenMode) {
            glutReshapeWindow(windowWidth, windowHeight);
            glutPositionWindow(windowPosX, windowPosY);
            // fullscreen mode is the best experience for sure, please test in F
            fullscreenMode = 0;
        }

        else {
            glutFullScreen();
            fullscreenMode = 1;
        }

        break;
    case 'q':
    case 'Q':

        cleanup();
        // cleanup function added last minute... realized i didnt have anything for mem management... VS Microsoft sent proper warning messages
        exit(0);
        break;

    case 'w':
    case 'W':
        moveForward = 1;
        break;

    case 's':
    case 'S':
        moveBackward = 1;
        break;

    case 'a':
    case 'A':
        moveLeft = 1;
        break;

    case 'd':
    case 'D':
        moveRight = 1;
        break;

    case 'b':
    case 'B':
        fogEnabled = !fogEnabled;
        break;

    default:
        break;
    }
}

void keyboardUpFunc(unsigned char key, int x, int y) {
    // build up from the initial keyboard action set up above, completes logic 
    switch (key) {
    // setting flags for movement logic
    case 'w':
    case 'W':
        moveForward = 0;
        break;

    case 's':
    case 'S':
        moveBackward = 0;
        break;

    case 'a':
    case 'A':
        moveLeft = 0;
        break;

    case 'd':
    case 'D':
        moveRight = 0;
        break;

    default:
        break;
    }
}

void specialKeys(int key, int x, int y) {
    // for downward and upward movement
    switch (key) {
    case GLUT_KEY_UP:
        moveUp = 1;
        break;
    case GLUT_KEY_DOWN:
        moveDown = 1;
        break;
    default:
        break;
    }
}

// Special keys release function
void specialKeysUp(int key, int x, int y) {
    switch (key) {
    case GLUT_KEY_UP:
        moveUp = 0;
        break;
    case GLUT_KEY_DOWN:
        moveDown = 0;
        break;
    default:
        break;
    }
}

void passiveMotionFunc(int x, int y) {
    // function controls camera movement, tried integrating built in pointer making the mouse built within the frame.
    if (prevMouseX == -1 || prevMouseY == -1) {
        prevMouseX = x;
        prevMouseY = y;
        return;
    }

    int dx = x - prevMouseX;
    int dy = y - prevMouseY;

    yAngle += dx * 0.005f;
    xAngle += dy * 0.005f;
    // camera andlge adjusted upon mouse movement interaction
    if (xAngle > M_PI / 2.0f - 0.1f)
        xAngle = M_PI / 2.0f - 0.1f;
    // added this after realizing the movement would invert reaching a yield down and up, stops this from happening
    if (xAngle < -M_PI / 2.0f + 0.1f)
        xAngle = -M_PI / 2.0f + 0.1f;

    prevMouseX = x;
    prevMouseY = y;
    // updates pos 

    glutPostRedisplay();
}

void drawoutSub() {
    // function draws out submarine mode 
    glPushMatrix();
    glTranslatef(submarinePosX, submarinePosY, submarinePosZ);
    // initial pos
    glRotatef(submarineYaw +  90.0f, 0.0f, 1.0f, 0.0f ); 
    // sets initial rotation angle properly based off asix offset position basically
    glScalef(0.05f, 0.05f, 0.05f); 
    // good scale 
    GLfloat materialAmbient[] = { 0.25f, 0.2f, 0.075f, 1.0f };
    GLfloat materialDiffuse[] = { 0.75f, 0.61f, 0.22648f, 1.0f };
    GLfloat materialSpecular[] = { 0.629f, 0.55f, 0.36f, 1.0f };
    // played around these, not really a specific num
    GLfloat materialShininess = 60.0f;
    glMaterialfv(GL_FRONT, GL_AMBIENT, materialAmbient);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, materialDiffuse);
    glMaterialfv(GL_FRONT, GL_SPECULAR, materialSpecular);
    glMaterialf(GL_FRONT, GL_SHININESS, materialShininess);
    // yeah queue yellow submarine by The Beatles
    glColor3f(1.0f, 1.0f, 0.0f);
    glBegin(GL_TRIANGLES);
    for (int i = 0; i < numOfFaces; i++) {
        // drawing out sub model
        int vIndicies[3];
        int nIndecies[3];
        // retrieving indices
        for (int j = 0; j < 3; j++) {
            vIndicies[j] = faces[i].v[j] - 1;
            // inital indices to start
            nIndecies[j] = faces[i].n[j] - 1;
        }
        for (int j = 0; j < 3; j++) {

            if (nIndecies[j] >= 0 && nIndecies[j]  < numOfNormals ) {
                glNormal3f(normals[nIndecies[j]].x, normals[nIndecies[j]].y, normals[nIndecies[j]].z );
                // setting normals based off indicies allocation
            }

            if (vIndicies[j] >= 0 && vIndicies[j] < numOfVertices) {
                glVertex3f(vertices[vIndicies[j]].x, vertices[vIndicies[j]].y, vertices[vIndicies[j]].z);
            }
        }
    }
    glEnd();
    glPopMatrix();
}

void drawSeaFloor() {
    // function handles the drawout of the base layer circle
    glPushMatrix();
    GLfloat viewEmission[] = { 0.3f, 0.3f, 0.3f, 1.0f };
    glMaterialfv(GL_FRONT, GL_EMISSION, viewEmission );

    glEnable(GL_TEXTURE_2D);
    // texturing
    glBindTexture(GL_TEXTURE_2D, baseTextID);
    glTranslatef(0.0f, -2.0f, 0.0f);
    // sets the position of the base 

    glRotatef(-90.0f, 1.0f, 0.0f, 0.0f);
    // for some reason had to be set like this, originally thought it out place out on the x axis horizontal automatically
    gluQuadricTexture(quadricDisk, GL_TRUE);
    // drawing out wiht texture
    gluDisk(quadricDisk, 0.0, 92.0, 50, 1);
    // dimensions

    glDisable(GL_TEXTURE_2D);
    // without this set the corals to same colour sometimes for some reason 

    GLfloat noEmission[] = { 0.0f, 0.0f, 0.0f, 1.0f };
    // attributed to not mess wiht lighting for base
    glMaterialfv(GL_FRONT, GL_EMISSION, noEmission);
    glPopMatrix();
}

void drawWallEnv() {
    // function draws out the vertical portion of the environment in a cylinder format, boids will be contained within this env
    glPushMatrix();

    GLfloat materialEmission[] = { 0.2f, 0.2f, 0.2f, 1.0f };
    glMaterialfv(GL_FRONT, GL_EMISSION, materialEmission );
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, cylinderTextureID);
    // same procedure as the base 

    glTranslatef(0.0f, -2.5f, 0.0f); 
    // lower position made the program start with the sub at a good spot... could also change this within the draw sub function but might as well here
    glRotatef(-90.0f, 1.0f, 0.0f, 0.0f);
    // proper positioning
    gluQuadricTexture(quadricCylinder, GL_TRUE);
    gluCylinder(quadricCylinder, 90.0, 90.0, cylinderHeight, 50, 1 );
    // draws out the cylinder... height matches the criteria with the assignment in being able to see the surface basically
    glDisable(GL_TEXTURE_2D);
    // makes sure to not extend texturing on other assets
    GLfloat noEmission[] = { 0.0f, 0.0f, 0.0f, 1.0f };
    glMaterialfv(GL_FRONT, GL_EMISSION, noEmission);
    // same procedure as disk

    glPopMatrix();
}

PPMIn* loadPPM(const char* filename) {
    // fucntion to laod in textures from PPM file format.. used a texture downloaded online 
    FILE* fp = fopen(filename, "rb");
    if (!fp) {
        printf("Failed to open PPM file %s\n", filename);
        return NULL;
    }

    char format[3];
    int width, height, maxVal;
    fscanf(fp, "%2s", format);
    // read in magic num and check format
    if (strcmp(format, "P6") == 0) {
        // end line setting
        while (fgetc(fp) != '\n' );
        // basic parsing
        while (fgetc(fp) == '#') {
            while (fgetc(fp) != '\n');
        }
        fseek(fp, -1, SEEK_CUR );
        // bitwise movement to allocate properly 
    }

    fscanf(fp, "%d %d %d", &width, &height, &maxVal );
    while (fgetc(fp) != '\n');
    // end of line

    unsigned char* data = (unsigned char*)malloc(width * height * 3);

    if (strcmp(format, "P6") == 0) {

        fread(data, 3, width * height, fp);
        // read in process
    }
    else if (strcmp(format, "P3") == 0) {\

        int i;

        for (i = 0; i < width * height * 3; i++) {

            int pixelAllocation;
            fscanf(fp, "%d", &pixelAllocation);
            data[i] = (unsigned char)pixelAllocation;
        }
    }
    fclose(fp);

    PPMIn* image = (PPMIn*)malloc(sizeof(PPMIn));

    image->data = data;

    image->width = width;

    image->height = height;

    return image;
}

GLuint loadTexture(const char* filename) {
    // fucntion to load in textures

    PPMIn* image = loadPPM(filename);
    if (!image) {
        printf("failed to process texture %s\n", filename);
        exit(1);
    }

    GLuint textureID;
    glGenTextures(1, &textureID);
    glBindTexture(GL_TEXTURE_2D, textureID);

    // settting texture params
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT );
    // horizontal vertical 
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT );  
    //  using mipmapping
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    // magnification oprocess
    gluBuild2DMipmaps( GL_TEXTURE_2D, GL_RGB, image->width, image->height,GL_RGB, GL_UNSIGNED_BYTE, image->data );
    // uploading textures with build 2d, looked at opengl documentation for ths 

    // Free image data
    free(image->data);
    free(image);

    return textureID;
}

void cleanup() {
    // basic cleanup procedure
    if (vertices) {
        free(vertices);
        vertices = NULL;
    }

    if (normals) {
        free(normals);
        normals = NULL;
    }

    if (faces) {
        free(faces);
        faces = NULL;
    }

    if (baseTextID) {
        glDeleteTextures(1, &baseTextID);
    }

    if (cylinderTextureID) {
        glDeleteTextures(1, &cylinderTextureID);
    }

    if (quadricDisk) {
        gluDeleteQuadric(quadricDisk);
        quadricDisk = NULL;
    }

    if (quadricCylinder) {
        gluDeleteQuadric(quadricCylinder);
        quadricCylinder = NULL;
    }

    if (waterVertices) {
        free(waterVertices);
        waterVertices = NULL;
    }

    if (waterIndices) {
        free(waterIndices);
        waterIndices = NULL;
    }

    if (CurFlock) {
        free(CurFlock);
        CurFlock = NULL;
    }

    if (PrevFlock) {
        free(PrevFlock);
        PrevFlock = NULL;
    }

    for (int i = 0; i < NUM_CORAL_MODELS; i++) {
        // sepoerate processing
        if (coralModels[i].vertices) {
            free(coralModels[i].vertices);
            coralModels[i].vertices = NULL;
        }
        if (coralModels[i].normals) {
            free(coralModels[i].normals);
            coralModels[i].normals = NULL;
        }
        if (coralModels[i].faces) {
            free(coralModels[i].faces);
            coralModels[i].faces = NULL;
        }
    }
}

// implementation for the corals to appear


void drawCorals() {
    glPushAttrib(GL_ALL_ATTRIB_BITS); 
    glEnable(GL_LIGHTING);
    glDisable(GL_TEXTURE_2D);
    // makes sure tesxturing is off for coral assets

    glDisable(GL_COLOR_MATERIAL);
    // allows abstraction for coral units, without it blends with colouring from boids

    for (int i = 0; i < NUM_CORALS; i++) {
        // enters loop to start coral placement process , generating multiple
        CoralInstances* coralUnit = &corals[i];
        glPushMatrix();
        glTranslatef( coralUnit->position[0], coralUnit->position[1], coralUnit->position[2]);
        glRotatef(coralUnit->rotation[1], 0.0f, 1.0f, 0.0f);
        //  set to act as a generation system for the corals

        glScalef(coralUnit->scale, coralUnit->scale, coralUnit->scale);

        // Set material properties (e.g., coral color)
        GLfloat materialAspect[] = { 0.2f, 0.1f, 0.1f, 1.0f };
        GLfloat materialDiffuse[] = { 0.8f, 0.4f, 0.4f, 1.0f };
        GLfloat materialSpecular[] = { 0.3f, 0.3f, 0.3f, 1.0f };
        GLfloat materialShininess = 20.0f;
        glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, materialAspect );
        // reference on setting from past experience in Cpp
        glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, materialDiffuse);
        glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, materialSpecular);
        glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, materialShininess);
        coralDrawout(coralUnit->model);
        // draws out model

        glPopMatrix();
    }
    glPopAttrib(); 
}


void coralModelLoad(const char* filename, CoralStructure* model) {
    // similar to the submarien obj load in but, tried using the same didnt quite work, wrote similarly but specified for multiple coral objects
    FILE* file = fopen(filename, "r");
    if (!file) {
        printf("cant open file %s\n", filename);
        exit(1);
    }
    char line[256];
    int vertexAllocation = 1000;
    int normalsAllocation = 1000;
    int facesAllocation = 1000;

    model->vertices = (CoralVerts*)malloc(sizeof(CoralVerts) * vertexAllocation);
    model->normals = (CoralNormal*)malloc(sizeof(CoralNormal) * normalsAllocation);
    model->faces = (CoralFace*)malloc(sizeof(CoralFace) * facesAllocation);
    // allocating mem for metrics

    model->numVertices = 0;

    model->numNormals = 0;

    model->numFaces = 0;

    while (fgets(line, sizeof(line), file)) {
        if (line[0] == 'v' && isspace(line[1])) {
            if (model->numVertices >= vertexAllocation) {
                //vertex allocation logic 
                vertexAllocation *= 2;
                model->vertices = (CoralVerts*)realloc(model->vertices, sizeof(CoralVerts) * vertexAllocation);
            }
            CoralVerts v;
            sscanf(line, "v %f %f %f", &v.x, &v.y, &v.z);
            model->vertices[model->numVertices++] = v;
        }
        else if (line[0] == 'v' && line[1] == 'n') {
            // setting for normals within obj
            if (model->numNormals >= normalsAllocation) {
                normalsAllocation *= 2;
                model->normals = (CoralNormal*)realloc(model->normals, sizeof(CoralNormal) * normalsAllocation);
            }
            CoralNormal n;
            sscanf(line, "vn %f %f %f", &n.x, &n.y, &n.z);
            model->normals[model->numNormals++] = n;
        }
        else if (line[0] == 'f' ) { 
            if (model->numFaces >= facesAllocation) {
                // and finally face allocation logic
                facesAllocation *= 2;
                model->faces = (CoralFace*)realloc(model->faces, sizeof(CoralFace) * facesAllocation);
            }
            CoralFace f;
            int matches = sscanf(line, "f %d//%d %d//%d %d//%d",
                &f.v[0], &f.n[0], &f.v[1], &f.n[1], &f.v[2], &f.n[2]);
            if (matches == 6) {
                model->faces[model->numFaces++] = f;
            }
            else {
                printf(" not able to preocess %s\n", line);
            }
        }
        else {
            // ignoring whitepsace and other comps 
        }
    }

    fclose(file);
}


void loadSubmarineModel(const char* filename) {
    // function loads in the sub, used referencing from the assignment 2 withnthe enterprise
    // and basically the same as the corals
    FILE* file = fopen(filename, "r");
    if (!file) {
        printf("cant open file  %s\n", filename);
        exit(1);
    }

    char line[256];
    int allocatedVertices = 1000;
    int allocatedNormals = 1000;
    int allocatedFaces = 1000;
    vertices = (Vertex*)malloc(sizeof(Vertex) * allocatedVertices );
    normals = (Normal*)malloc(sizeof(Normal) * allocatedNormals);
    faces = (Face*)malloc(sizeof(Face) * allocatedFaces);
    numOfVertices = 0;
    numOfNormals = 0;
    numOfFaces = 0;

    while (fgets(line, sizeof(line), file)) {
        if (line[0] == 'v' && line[1] == ' ') {
            if (numOfVertices >= allocatedVertices) {
                allocatedVertices *= 2;
                vertices = (Vertex*)realloc(vertices, sizeof(Vertex) * allocatedVertices);
            }
            Vertex v;
            sscanf(line, "v %f %f %f", &v.x, &v.y, &v.z);
            vertices[numOfVertices++] = v;
        }
        else if (line[0] == 'v' && line[1] == 'n' ) {
            if (numOfNormals >= allocatedNormals) {
                allocatedNormals *= 2;
                normals = (Normal*)realloc(normals, sizeof(Normal) * allocatedNormals );
            }
            Normal n;
            sscanf(line, "vn %f %f %f", &n.x, &n.y, &n.z);
            normals[numOfNormals++] = n;
        }
        else if (line[0] == 'f') {
            if (numOfFaces >= allocatedFaces) {
                allocatedFaces *= 2;
                faces = (Face*)realloc(faces, sizeof(Face) * allocatedFaces );
            }
            Face f;
            int matches = sscanf(line, "f %d//%d %d//%d %d//%d",
                &f.v[0], &f.n[0], &f.v[1], &f.n[1], &f.v[2], &f.n[2]);
            if (matches == 6) {
                faces[numOfFaces++] = f;
            }
            else {
                printf("not recognized: %s\n", line);
            }
        }
        else {

        }
    }

    fclose(file);
}

void initializeCorals() {
    // Load coral models
    char filename[256];
    for (int i = 0; i < NUM_CORAL_MODELS; i++) {
        sprintf(filename, "coral/coral_%d.obj", i + 1);
        coralModelLoad(filename, &coralModels[i]);
    }

    // Define cluster centers for grouping
#define NUM_CLUSTERS 3
    float clusterCenters[NUM_CLUSTERS][2] = {
        { 20.0f, 20.0f },
        { -30.0f, -10.0f },
        { 10.0f, -40.0f }
    };

    // Initialize coral instances
    srand((unsigned int)time(NULL)); // Seed random number generator

    for (int i = 0; i < NUM_CORALS; i++) {
        // Assign a random coral model to this instance
        corals[i].model = &coralModels[rand() % NUM_CORAL_MODELS];

        float r = ((float)rand() / RAND_MAX);
        if (r < 0.6f) {
            // Place in a cluster (60% chance)
            int clusterIndex = rand() % NUM_CLUSTERS;
            float clusterX = clusterCenters[clusterIndex][0];
            float clusterZ = clusterCenters[clusterIndex][1];

            // Random offset within cluster
            float offsetX = (((float)rand() / RAND_MAX) - 0.5f) * 10.0f; // +/-5 units
            float offsetZ = (((float)rand() / RAND_MAX) - 0.5f) * 10.0f;

            corals[i].position[0] = clusterX + offsetX;
            corals[i].position[2] = clusterZ + offsetZ;
        }
        else {
            // Place randomly (40% chance)
            float angle = ((float)rand() / RAND_MAX) * 2.0f * M_PI;
            float radius = ((float)rand() / RAND_MAX) * 80.0f;
            // easily adjustable generation right now gives out a good amount of corals givent eh size of my environment

            corals[i].position[0] = radius * cosf(angle);
            corals[i].position[2] = radius * sinf(angle);
        }

        // Position on the ocean floor
        corals[i].position[1] = -2.0f; // Adjust Y position as needed

        // Random rotation around Y-axis
        corals[i].rotation[0] = 0.0f;
        corals[i].rotation[1] = ((float)rand() / RAND_MAX) * 360.0f;
        corals[i].rotation[2] = 0.0f;

        // have to set randomized scale initially a bit bigger they were like miniature scale initially lol
        corals[i].scale = 9.0f + ((float)rand() / RAND_MAX) * 10.0f; 
    }
}


void coralDrawout(CoralStructure* model) {
    // function to drawout corals 
    glBegin(GL_TRIANGLES);
    for (int i = 0; i < model->numFaces; i++) {
        // Get indices
        int vIndex[3];
        int nIndex[3];
        for (int j = 0; j < 3; j++) {
            vIndex[j] = model->faces[i].v[j] - 1;
            nIndex[j] = model->faces[i].n[j] - 1;
        }

        // Set normals and vertices
        for (int j = 0; j < 3; j++) {
            if (nIndex[j] >= 0 && nIndex[j] < model->numNormals) {
                glNormal3f(model->normals[nIndex[j]].x, model->normals[nIndex[j]].y, model->normals[nIndex[j]].z);
            }
            if (vIndex[j] >= 0 && vIndex[j] < model->numVertices) {
                glVertex3f(model->vertices[vIndex[j]].x, model->vertices[vIndex[j]].y, model->vertices[vIndex[j]].z);
            }
        }
    }
    glEnd();
}


/*

BONUS POINT OPTION:

D Boids Logic implementation, going for the 19 / 15 option for the fish, extending my logic from assignment one, again as mentioned in 1, very challenging but fun to work on

*/


void initializeBoids() {
    CurFlock = (Boid*)malloc(sizeof(Boid) * NUMBER_OF_BOIDS);
    // allocating mem for flocks 
    PrevFlock = (Boid*)malloc(sizeof(Boid) * NUMBER_OF_BOIDS);

    srand((unsigned int)time(NULL));
    for (int i = 0; i < NUMBER_OF_BOIDS; i++) {
        // setting boids individually, contained within env
        float initAngle = ((float)rand() / RAND_MAX) * 2.0f * M_PI;
        float setRad = ((float)rand() / RAND_MAX) * (100.0f - WALL_YIELD_STEER );
        // randomized allocation into the program
        CurFlock[i].position[0] = setRad * cosf(initAngle);
        CurFlock[i].position[2] = setRad * sinf(initAngle);
        // allocating x y coords to based off angle and radius setting, for position
        CurFlock[i].position[1] = ((float)rand() / RAND_MAX) * (cylinderHeight - 2 * WALL_YIELD_STEER) + (-2.5f + WALL_YIELD_STEER );
        float intiSpeed = MIN_SPEED + ((float)rand() / RAND_MAX) * (MAX_SPEED - MIN_SPEED);
        // randomized speed generation
        float thetaDirection = ((float)rand() / RAND_MAX) * 2.0f * M_PI;
        float phiDirection = ((float)rand() / RAND_MAX) * M_PI - M_PI / 2.0f;
        // generating randomized directions
        CurFlock[i].velocity[0] = intiSpeed * cosf(phiDirection) * cosf(thetaDirection); 

        CurFlock[i].velocity[1] = intiSpeed * sinf(phiDirection);

        CurFlock[i].velocity[2] = intiSpeed * cosf(phiDirection) * sinf(thetaDirection);
        //  velocity component computation based off of the spherical coords x, y,z components accordingly
    }
}



void boidLogic() {
    // enhancement from assingment 1 to apply towards A 3d environment
    for (int i = 0; i < NUMBER_OF_BOIDS; i++) {
        PrevFlock[i] = CurFlock[i];
        // same flock copying logic from as 1 
    }

    float cylinderRadius = 90.0f;
    // setting to same (ish) metric as actrual environments border
    for (int i = 0; i < NUMBER_OF_BOIDS; i++) {
        // setting pointer for boid index
        Boid* boidUnit = &CurFlock[i];
        float distances[NUMBER_OF_BOIDS];
        int neighborIndices[NUMBER_OF_BOIDS];
        // allocation for neighors

        for (int j = 0; j < NUMBER_OF_BOIDS; j++) {
            if (j == i) {
                distances[j] = INFINITY;
                // cannot set self neighbour for boid unit
                continue;
            }

            float dx = PrevFlock[j].position[0] - boidUnit->position[0];
            // computing diff between j boid and current boidUnit
            float dy = PrevFlock[j].position[1] - boidUnit->position[1];
            float dz = PrevFlock[j].position[2] - boidUnit->position[2];
            distances[j] = sqrtf(dx * dx + dy * dy + dz * dz);
            // previously used logic from as 1, euclidean distance
        }

        for (int j = 0; j < NUMBER_OF_BOIDS; j++) {
            // initializing neighbour ind
            neighborIndices[j] = j;
        }
        for (int j = 0; j < NUMBER_OF_BOIDS - 1; j++) {
            // sorting set neighbours based off dist
            for (int k = j + 1; k < NUMBER_OF_BOIDS; k++) {
                if (distances[j] > distances[k]) {
                    float tempDist = distances[j];
                    distances[j] = distances[k];
                    distances[k] = tempDist;
                    // bubble sort sorting for allocating neighbours distance
                    int tempIndex = neighborIndices[j];
                    neighborIndices[j] = neighborIndices[k];
                    // indice swapping
                    neighborIndices[k] = tempIndex;
                }
            }
        }
        float averageVelocity[3] = { 0.0f, 0.0f, 0.0f  };

        int indxCount = 0;
        for (int j = 0; j < NEIGHBORS; j++ ) {

            int index = neighborIndices[j];

            averageVelocity[0] += PrevFlock[index].velocity[0];

            averageVelocity[1] += PrevFlock[index].velocity[1];

            averageVelocity[2] += PrevFlock[index].velocity[2];
            indxCount++;
            // getting average velocity of set ner neaighbours 
        }

        if (indxCount > 0) {
            // getting avg velocity components
            averageVelocity[0] /= indxCount;

            averageVelocity[1] /= indxCount;

            averageVelocity[2] /= indxCount;
        }

        boidUnit->velocity[0] +=  (averageVelocity[0] - boidUnit->velocity[0]) *  0.06f ;
        boidUnit->velocity[1] += (averageVelocity[1] - boidUnit->velocity[1]) * 0.06f; 

        boidUnit->velocity[2] += (averageVelocity[2] - boidUnit->velocity[2]) *  0.06f;
        // adjusting the boid velocity based off of average from neighbour values

        float separationForce[3] = { 0.0f, 0.0f, 0.0f };
        // separation logic, based off push from separation vect to avoid crowding issues, had tomrework a lot compared to assignment 1 to work properly
        for (int j = 0; j < NEIGHBORS; j++) {
            int curCount = neighborIndices[j];
            float diffX = boidUnit->position[0] - PrevFlock[curCount].position[0];
            float diffY = boidUnit->position[1] - PrevFlock[curCount].position[1];
            float diffZ = boidUnit->position[2] - PrevFlock[curCount].position[2];
            // getting diff between cur boid and neighbour, asme sorta process
            float distanceSep = sqrtf(diffX * diffX + diffY * diffY + diffZ * diffZ );
            if (distanceSep < SEPARATION && distanceSep > 0.0f) {
                separationForce[0] += diffX / (distanceSep * distanceSep);
                separationForce[1] += diffY / (distanceSep * distanceSep);
                // force decrease procedure
            }
        }
        boidUnit->velocity[0] += separationForce[0];
        boidUnit->velocity[1] += separationForce[1];
        boidUnit->velocity[2] += separationForce[2];

        float centerDistance = sqrtf(boidUnit->position[0] * boidUnit->position[0] + boidUnit->position[2] * boidUnit->position[2]);
        // logic set for wall avoidance, sets steerign for to push away
        if (centerDistance > (cylinderRadius - WALL_YIELD_STEER)) {
            // check procedure for how much boid is nearing/into boundary
            float pushOffYield = (centerDistance - (cylinderRadius - WALL_YIELD_STEER)) / WALL_YIELD_STEER;
            boidUnit->velocity[0] -= pushOffYield * (boidUnit->position[0] / centerDistance );
            boidUnit->velocity[2] -= pushOffYield * (boidUnit->position[2] / centerDistance);
            // applies force proportionally to the penetration yield
        }
        float groundLevel = -2.5f;
        // value same as seafloor metric, avoidance for collision 
        if (boidUnit->position[1] < groundLevel + WALL_YIELD_STEER) {
            float pushFactor = (groundLevel + WALL_YIELD_STEER - boidUnit->position[1]) / WALL_YIELD_STEER;
            boidUnit->velocity[1] += pushFactor;
        }

        float waterSurfaceLevel = groundLevel + cylinderHeight;
        // check for vertical boundary separation/collision
        if (boidUnit->position[1] > waterSurfaceLevel - WALL_YIELD_STEER) {
            // check how much boid is nearing/into boundary settings
            float pushFactor = (boidUnit->position[1] - (waterSurfaceLevel - WALL_YIELD_STEER)) / WALL_YIELD_STEER;
            // again, sets opposite push upwards reverting the movement
            boidUnit->velocity[1] -= pushFactor;
        }

        float velocitiesMagnitude = sqrtf(boidUnit->velocity[0] * boidUnit->velocity[0] +  boidUnit->velocity[1] * boidUnit->velocity[1] + boidUnit->velocity[2] * boidUnit->velocity[2] );
        // computation fro the magnitude of the boids velocity vect, then scales back to set max
        if (velocitiesMagnitude > 0.0f) {

            boidUnit->velocity[0] = (boidUnit->velocity[0] / velocitiesMagnitude) * MAX_SPEED;

            boidUnit->velocity[1] = (boidUnit->velocity[1] / velocitiesMagnitude) * MAX_SPEED;

            boidUnit->velocity[2] = (boidUnit->velocity[2] / velocitiesMagnitude) * MAX_SPEED;
            // division by zero case check avoidance, sets scale
        }
        boidUnit->position[0] += boidUnit->velocity[0];
        // updating boid units positions
        boidUnit->position[1] += boidUnit->velocity[1];
        boidUnit->position[2] += boidUnit->velocity[2];
        // set based off velocity
    }
}

void drawoutBoids() {
    for (int i = 0; i < NUMBER_OF_BOIDS; i++) {\
        // initiates individual allocation process into the environment
        Boid* boid = &CurFlock[i];

        glPushMatrix();
        glTranslatef(boid->position[0], boid->position[1], boid->position[2]);

        // attemptsnm to make the  boid to face its velocity direction
        float angleY = atan2f(boid->velocity[0], boid->velocity[2] ) * 180.0f / M_PI;
        float angleX = asin(boid->velocity[1] / MAX_SPEED ) * 180.0f  / M_PI;
        glRotatef(angleY, 0.0f, 1.0f, 0.0f);
        glRotatef(-angleX, 1.0f, 0.0f, 0.0f);

        glScalef(3.0f, 1.5f, 1.5f); 
        // fishes size
        // adjustable, size right now seems about right
        glColor3f(1.0f, 0.0f, 1.0f); 
        //glColor3f(0.5, 1.0f, 0.5f);
        // purple fishies, set rn, tested different looks

        glBegin(GL_TRIANGLES);
        glVertex3f(1.0f, 0.0f, 0.0f); 
        glVertex3f(-1.0f, 0.5f, 0.0f);  
        glVertex3f(-1.0f, -0.5f, 0.0f); 
        // supposed to be a triangle mimicking the fish 
        glEnd();

        glPopMatrix();
    }
}


// same sort of terminal based command explanation as the two assingments
void printKeyboardControls()
{
    printf("\n");
    printf("Program Controls:\n");
    printf("*************************************************************************\n");
    printf("\n");
    printf("W     :  Move forward\n");
    printf("S     :  Move Backwards\n");
    printf("D     :  Rotate (Spin) Right\n");
    printf("A     :  Rotate (Spin) Left\n");
    printf("Up Arrow  :  Move upwards\n");
    printf("Down Arrow:  Move Downwards\n");
    printf("\n");
    printf("Note: I enhanced the movement to interact with the orientation of the sub, meaning while idle A and D will rotate accordingly");
    printf("but will turn accordingly when moving forward or backwards... Makes the movemnet more interactive!\n");
    printf("\n");
    printf("B     :  Toggle Fog\n");
    printf("F     :  Toggle Fullscreen mode\n");
    printf("U     :  Toggle Wireframe mode\n");
    printf("Q     :  Quit the program\n");
    printf("\n");
    printf("Use the mouse to control the camera accordingly, play around with it!\n");
}
















