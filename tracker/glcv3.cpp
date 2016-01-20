#include "opencv2/opencv.hpp"
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include <cstdio>
#include <cstdlib>
#include <math.h>
#include <GL/glut.h>
#include <sys/time.h>
#define WIDTH 640
#define HEIGHT 480
#define ASPECT 1.33333333
#define PATTERNSIZE 200

#define MAT(a,b,c) CV_MAT_ELEM(a, float, b, c)
#define forMat(i, j, mat) for (int i = 0; i < mat.rows; i++) for (int j = 0; j < mat.cols; j++)

#define FCL 2 
#define ZD (0)
using namespace cv;
using namespace std;
Mat cam, R, T, p3, p2, rotMat, tMat, tr;
//CvMat *cam, *p3, *R, *T, *p2, *rotMat, *tMat, *p2_, *H;

int drawOverlay = 1;
int useFlow = 0;

float camDat [3][3] = {
  {FCL, 0, 0},
  {0, FCL*ASPECT, 0},
  {0, 0, 1}};

float p3Dat[4][3] = {
  {-1, -1, ZD},
  {1,  -1, ZD},
  {1,   1, ZD},
  {-1,  1, ZD}};

float p2Dat[4][2] = {
  {-0.375, -0.5 },
  {0.375, -0.5},
  {0.375, 0.5 },
  {-0.375, 0.5 }};
  
float p2_Dat[4][2] = {
  {0, 0},
  {PATTERNSIZE-1, 0},
  {PATTERNSIZE-1, PATTERNSIZE-1},
  {0, PATTERNSIZE-1}};

VideoCapture capture; // open the default camera
GLuint cameraImage;
Mat frame, oldGray;
vector<Point2f> corners;

IplImage *pattern;

double timestamp() {
  timeval start; 
  gettimeofday(&start, NULL);
  return ((start.tv_sec) + start.tv_usec/1000000.0);
}

void loadTexture_Ipl(Mat &image, GLuint *text) {
  glGenTextures(1, text);

  glBindTexture( GL_TEXTURE_2D, *text ); //bind the texture to it's array
  glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
 
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
 
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, image.cols, image.rows, 0,  GL_BGR, GL_UNSIGNED_BYTE, &image.at<Vec3b>(0, 0)[0]);
}
void findExtrinsic() {
  Mat dist = Mat::zeros(4, 1, CV_32FC1);
  //solvePnP(p3, p2, cam, dist, R, T, 0, CV_EPNP);
  solvePnP(p3, p2, cam, dist, R, T, 0);

  //setMat(p2, (float*)p2Dat);
  //cvFindExtrinsicCameraParams2(p3, p2, cam, NULL, R, T);
  
  Rodrigues(R, rotMat);
  
  for(int i = 0; i < 3; i++) {
    for(int j = 0; j < 3; j++) {
      tMat.at<float>(i, j) = rotMat.at<double>(i, j);
    }
    tMat.at<float>(i, 3) = T.at<double>(i, 0);
  }
  tMat.at<float>(3, 0) = tMat.at<float>(3, 1) = tMat.at<float>(3, 2) = 0;
  tMat.at<float>(3, 3) = 1;

  forMat (i, j, tr) {
    tr.at<float>(i, j) = tMat.at<float>(j, i);
  }
}
inline int bRange(int a, int l, int h) {
  if (a > h) a = h;
  if (a < l) a = l;
  return a;
}
void bilinearInterpolate(IplImage *out, int inr, int inc, IplImage * image, float r, float c) {
  int ir = bRange(r, 0, image->height-1);
  int ic = bRange(c, 0, image->width-1);
  int ir1 = bRange(r+1, 0, image->height-1);
  int ic1 = bRange(c+1, 0, image->width-1);
  
  float dr = r-ir, dc = c-ic;
  float dr_ = 1-dr, dc_ = 1-dc;
  
  float val = 0;
  static float con[3] = {0.11, 0.59, 0.3};
  for(int i=0;i<3;i++) {
    val += con[i] * ( (CV_IMAGE_ELEM(image, uchar, ir, ic*3  + i)*dr_ 
              + CV_IMAGE_ELEM(image, uchar, ir1, ic*3  + i)*dr)*dc_ 
              + (CV_IMAGE_ELEM(image, uchar, ir, ic1*3 + i)*dr_ 
              + CV_IMAGE_ELEM(image, uchar, ir1, ic1*3 + i)*dr)*dc);
    
  }
  for(int i=0;i<3;i++) 
    CV_IMAGE_ELEM(out, uchar, inr, inc*3+i) = val;
}
void applyRotate(int type, int &x, int &y) {
  static int mat[4][2][2] = {
    {{1,0},{0,1}},
    {{0,1},{-1,0}},
    {{-1,0},{0,-1}},
    {{0,-1},{1,0}},
  };
  x -= (PATTERNSIZE-1) * 0.5;
  y -= (PATTERNSIZE-1) * 0.5;
  int nx = x*mat[(4+type)%4][0][0] + y*mat[(4+type)%4][1][0];
  int ny = x*mat[(4+type)%4][0][1] + y*mat[(4+type)%4][1][1];
  x = nx + (PATTERNSIZE-1) * 0.5;
  y = ny + (PATTERNSIZE-1) * 0.5;
}
int findOrientation() {
  int h = (PATTERNSIZE-1) * 0.5;
  int min = 1e6, output = 0;
  for(int i=0;i<4;i++) {
    float val = 0;
    for(int x = h-10; x <= h+10; x++) {
      int py = 35, px = x;
      applyRotate(i, px, py);
      val += CV_IMAGE_ELEM(pattern, uchar, py, px*3); 
    }
    if(val < min) {
      output = i;
      min = val;
    }
  }
  return output;
}
/*
void findHomography() {
  cvFindHomography(p2_, p2, H);
  
  for(int i=0;i<PATTERNSIZE;i++) {
    for(int j=0;j<PATTERNSIZE;j++) {
      float d = MAT(*H, 2, 0) * i  + MAT(*H, 2, 1) * j + MAT(*H, 2, 2);
      
      float ii = (MAT(*H, 0, 0) * i  + MAT(*H, 0, 1) * j + MAT(*H, 0, 2)) / d;
      float jj = (MAT(*H, 1, 0) * i  + MAT(*H, 1, 1) * j + MAT(*H, 1, 2)) / d;
      ii = (ii+1) * WIDTH / 2;
      jj = (jj+1) * HEIGHT / 2;
      bilinearInterpolate(pattern, j, i, frame, jj, ii);
    }
  }
  int orient = findOrientation();
  //cvSmooth(pattern, pattern);
  for(int i=0;i<PATTERNSIZE;i++) {
    for(int j=0;j<PATTERNSIZE;j++) {
      int px = j, py = i;
      applyRotate(orient, px, py); 
      for(int k=0;k<3;k++) {
        CV_IMAGE_ELEM(frame, uchar, i, j*3 + k) = CV_IMAGE_ELEM(pattern, uchar, py, px*3 + k);
      }
    }
  }
  
}*/
void loop() {
  capture >> frame;
  Mat gray; 
  cvtColor(frame, gray, CV_BGR2GRAY);

  double start = timestamp();
  if (useFlow) {
    vector<Point2f> newCorners;
    Mat status, err;
    calcOpticalFlowPyrLK(oldGray, gray, corners, newCorners, status, err);
    corners = newCorners;
  } else {
    Mat grayFilter, thresh; 
    cvtColor(frame, gray, CV_BGR2GRAY);
    medianBlur(gray, grayFilter, 5);
    threshold(grayFilter, thresh, 50, 255, CV_THRESH_BINARY);

    //Canny(gray, thresh, 100, 200, 3);
    vector<vector<Point> > contours;
    vector<vector<Point> > contours2;
    vector<Vec4i> hierarchy;
    findContours(thresh, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
    //Mat drawing = Mat::zeros( thresh.size(), CV_8UC3 );

    RNG rng(12345);
    contours2.resize(contours.size());
    float biggestArea = 0;
    int id = 0;
    Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
    for( int i = 0; i< contours.size(); i++ ) {
      //approxPolyDP(Mat(contours[i]), contours2[i], arcLength(contours[i], true) * 0.02, true);
      approxPolyDP(Mat(contours[i]), contours2[i], 4, true);
      //drawContours(frame, contours2, i, color, 1, 8, hierarchy, 0, Point() );
      if (contours2[i].size() == 4) {
        float area = contourArea(contours2[i]);
        if (area > biggestArea && area < 0.5 * frame.rows * frame.cols) {
          id = i;
          biggestArea = area;
          for (int j = 0; j < 4; j++) {
            corners[j].x = contours2[i][j].x; 
            corners[j].y = contours2[i][j].y; 
          }
        }
      }
      //if (contour.size())
      //drawContours(frame, contour, i, color, 2, 8, hierarchy, 0, Point() );
    }
  }
  printf("%f\n", timestamp() - start);
  oldGray = gray;
  cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.01));
  for (int i = 0; i < 4; i++) {
    circle(frame, corners[i], 2, Scalar(0, 0, 255));
  }
  for(int i=0;i<4;i++) {
    p2.at<float>(i, 0) = 1 - corners[3-i].x * 2.0 / frame.cols;
    p2.at<float>(i, 1) = -(corners[3-i].y * 2.0 / frame.rows -1);
  }

  //drawContours(frame, contours2, id, color, 2, 8, hierarchy, 0, Point() );

  //imshow("draw", drawing);
  /*
  int squares=0;
  CvPoint c[4];
  float biggestArea = 0;
  
  int wxh = frame->width*frame->height;
  for(CvSeq* a = contour; a; a = a->h_next ) {
      CvSeq *result = cvApproxPoly( a, sizeof(CvContour), storage2,
                  CV_POLY_APPROX_DP, cvContourPerimeter(a)*0.02, 0 );
                //printf("%d\n",result->total);
      float area = fabs(cvContourArea(result,CV_WHOLE_SEQ));
      cvDrawContours( threshBGR, a, CV_RGB(0,255,0), CV_RGB(255,0,0), 0, 1, 8 );
      if( result->total == 4 &&
          area > 1000 && area < wxh * 0.5 &&
          //cvCheckContourConvexity(result) &&
          area > biggestArea) {
        
        
        biggestArea = area;
        for(int i=0;i<4;i++) {
          corners[i].x = ((CvPoint*)cvGetSeqElem(result, i))->x;
          corners[i].y = ((CvPoint*)cvGetSeqElem(result, i))->y;
        }
//          c[i] = *(CvPoint*)cvGetSeqElem(result, i);
        squares++;
        //printf("%f\n", area);
      }
      if(result) {
        cvClearSeq( result );
        result = 0;
      }
      
  }
  if(biggestArea > 0) {
    //cvDrawContours( frame, biggest, CV_RGB(0,255,0), CV_RGB(255,0,0), 0, 1, 8 );
    //cvLine(frame, c[0], c[1],  CV_RGB(0,255,0), 1, 8);
    //cvLine(frame, c[1], c[2],  CV_RGB(0,255,0), 1, 8);
    //cvLine(frame, c[2], c[3],  CV_RGB(0,255,0), 1, 8);
    //cvLine(frame, c[3], c[0],  CV_RGB(0,255,0), 1, 8);
//    cvFindCornerSubPix
    cvFindCornerSubPix( grayFrameFiltered, corners, 4, cvSize( 11, 11 ), 
			cvSize( -1, -1 ), cvTermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
			
    for(int i=0;i<4;i++) {
      p2Dat[i][0] = corners[3-i].x *2.0 / WIDTH -1;
      p2Dat[i][1] = -(corners[3-i].y *2.0 / HEIGHT -1);
    }
  }

  cvClearSeq(contour);
  cvReleaseMemStorage(&storage1);
  cvReleaseMemStorage(&storage2);
  
  //cvFlip(threshBGR);
  

 */ 
  findExtrinsic();
  //findHomography();

  flip(frame, frame, 0);
  flip(frame, frame, 1);
  loadTexture_Ipl(frame, &cameraImage);
  glutPostRedisplay();
}
void drawBackground() {
   glMatrixMode(GL_PROJECTION);
   glLoadIdentity();

   glMatrixMode(GL_MODELVIEW);
   glLoadIdentity();
   
   glEnable( GL_TEXTURE_2D );
   glBindTexture( GL_TEXTURE_2D, cameraImage);
   
   glColor3f(1.0,1.0,1.0);
   //glDisable(GL_LIGHTING);
   glBegin( GL_QUADS );
   glTexCoord2d(0.0,0.0); glVertex2d(-1.0,-1.0);
   glTexCoord2d(1.0,0.0); glVertex2d(1.0,-1.0);
   glTexCoord2d(1.0,1.0); glVertex2d(1.0,1.0);
   glTexCoord2d(0.0,1.0); glVertex2d(-1.0,1.0);
   glEnd();
   glDisable( GL_TEXTURE_2D );
   //glEnable(GL_LIGHTING);
   
   // VERY IMPORTANT
   glDeleteTextures(1, &cameraImage);
}

void display(void) {
  glViewport(0, 0, WIDTH, HEIGHT);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  drawBackground();
  
  glClear(GL_DEPTH_BUFFER_BIT);

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  GLfloat a[16] = {
    FCL,  0,  0, 0,
    0,  FCL*ASPECT,  0, 0,
    0, 0, 1, 1,
    0,  0,  -1, 0 };
  glMultMatrixf(a);
  
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glMultMatrixf(&tr.at<float>(0, 0));
  GLfloat aa[16] = {
    1.01,  0,  0, 0,
    0,  1.01,  0, 0,
    0, 0, 1, 0,
    0,  0,  0, 1 };
  //glMultMatrixf(aa);
  
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  if(drawOverlay) {
    glColor4f(1.0,0,0,0.5);

/*    static GLfloat red_diff[]    = { 1.0 ,0,0,0.5 };
    glMaterialfv(GL_FRONT, GL_AMBIENT, red_diff);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, red_diff);
    glMaterialfv(GL_FRONT, GL_SPECULAR, red_diff);*/
    float x = -1;
    
    glBegin(GL_TRIANGLE_STRIP);
    glVertex3f(-1, -1, ZD);
    glVertex3f(1, -1, ZD);
    glVertex3f(-1, 1, ZD);
    glVertex3f(1, 1, ZD);
    glEnd();
  
    glColor4f(0,1.0,0,0.5);
    glBegin(GL_TRIANGLE_STRIP);
    glVertex3f(-1, -1, ZD-0.6*x);
    glVertex3f(1, -1, ZD-0.6*x);
    glVertex3f(-1, 1, ZD-0.6*x);
    glVertex3f(1,  1, ZD-0.6*x);
    glEnd();
  
    glColor4f(0,0,1.0,0.5);
    glBegin(GL_TRIANGLE_STRIP);
    glVertex3f(-1, -1, ZD-1.2*x);
    glVertex3f(1, -1, ZD-1.2*x);
    glVertex3f(-1, 1, ZD-1.2*x);
    glVertex3f(1,  1, ZD-1.2*x);
    glEnd();
    
    glColor4f(0,1.0,1.0,0.5);
    glBegin(GL_TRIANGLE_STRIP);
    glVertex3f(-1, -1, ZD-1.8*x);
    glVertex3f(1, -1, ZD-1.8*x);
    glVertex3f(-1, 1, ZD-1.8*x);
    glVertex3f(1,  1, ZD-1.8*x);
    glEnd();
    
    glColor4f(1.0,1.0,0,0.5);
    glBegin(GL_TRIANGLE_STRIP);
    glVertex3f(-1, -1, ZD-2.4*x);
    glVertex3f(1, -1, ZD-2.4*x);
    glVertex3f(-1, 1, ZD-2.4*x);
    glVertex3f(1,  1, ZD-2.4*x);
    glEnd();
  }
  glFlush();
}

void changeSize(int w, int h) {
  //width = w;
  //height = h;
  //glutPostRedisplay();
}

void keyboard(unsigned char key, int x, int y) {
  if (key == 'f') {
    useFlow ^= 1;
    printf("use flow = %d\n", useFlow);
  }
}

int main(int argc, char **argv) {

  corners.resize(4);
  cam = Mat(3, 3, CV_32FC1, &camDat);
  p3 = Mat(4, 3, CV_32FC1, &p3Dat);
  p2 = Mat(4, 2, CV_32FC1, &p2_Dat);
 
  R = Mat(3, 1, CV_32FC1);
  T = Mat(3, 1, CV_32FC1);
  rotMat = Mat(3, 3, CV_32FC1);
  tr = Mat::eye(4, 4, CV_32FC1);
  tMat = Mat(4, 4, CV_32FC1);
  
  //H = cvCreateMat(3, 3, CV_32FC1);
  pattern = cvCreateImage(cvSize(PATTERNSIZE, PATTERNSIZE), 8, 3);
  findExtrinsic();

      
  frame = cvCreateImage( cvSize(WIDTH, HEIGHT), 8, 3 );
  capture = VideoCapture(0);
  capture.set(CV_CAP_PROP_EXPOSURE, 0);
  capture.set(CV_CAP_PROP_FRAME_WIDTH, 640);
  capture.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
  capture.set(CV_CAP_PROP_FPS, 60);
  
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB | GLUT_DEPTH);
  glutInitWindowSize(WIDTH, HEIGHT);
  glutCreateWindow("Finding extrinsic");
  glEnable(GL_DEPTH_TEST);
  
  static GLfloat light_ambient[]  = { 0.5, 0.5, 0.5, 1.0 };
  static GLfloat light_diffuse[]  = { 1.0, 1.0, 1.0, 1.0 };
  static GLfloat light_specular[] = { 0.0, 0.0, 0.0, 1.0 };
  static GLfloat light_position[] = { 0.0, 0.5, 1.0, 0.0 };

  static GLfloat mat_ambient[]    = { 0.7, 0.7, 0.65, 1.0 };
  static GLfloat mat_diffuse[]    = { 0.5, 0.5, 0.5, 1.0 };
  static GLfloat mat_specular[]   = { 0.0, 0.0, 0.0, 1.0 };
  static GLfloat high_shininess[] = { 0 };
  
  glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
  glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
  glLightfv(GL_LIGHT0, GL_POSITION, light_position);


  glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ambient);
  glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_diffuse);
  glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
  glMaterialfv(GL_FRONT, GL_SHININESS, high_shininess);

  //glEnable(GL_LIGHTING);
  //glEnable(GL_LIGHT0);
  
  glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
  
  glutDisplayFunc(display);
  glutReshapeFunc(changeSize);
  glutIdleFunc(loop);
	glutKeyboardUpFunc(keyboard);
  glutMainLoop();
  return 0;
}

