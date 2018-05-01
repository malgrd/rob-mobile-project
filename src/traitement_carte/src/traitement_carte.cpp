#include <stdio.h>
#include <stdlib.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <iostream>
//ATTENTION AJOUT ROS
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int32.h>

//Publisher point
ros::Publisher pub_point_0;
ros::Publisher pub_largeur_img;

//Variable a envoyer
geometry_msgs::Point ptAEnvoyer;
std_msgs::Int32 nbLignes;

using namespace cv;
using namespace std;

int main (int argc, char* argv[])
//Ligne à compiler : g++ traitement_carte.cpp -o traitement_carte `pkg-config --cflags --libs opencv`
{
	Mat imageInit;
	Mat imageErodee;
	Mat imageDilatee;
	Mat imageFinale;
	
	
    imageInit = imread("/home/marion/catkin_ws/src/robmobile_projet/maps/my_map_simu_gazebo.pgm", CV_LOAD_IMAGE_GRAYSCALE);   // Read the file
    /*namedWindow( "Image originale", WINDOW_NORMAL);// Create a window for display.
    imshow( "Image originale", imageInit);                   // Show our image inside it.
    waitKey(0);*/
    
   
    
    if(imageInit.empty() ) // Check for invalid input
    {
        std::cout <<  "Could not open or find the image" << std::endl ;
        return -1;
    }
	
	//On erode et on dilate l'image pour faire une ouverture
	//erode(imageDep,imageArrivee, Mat(),nbIterations,
	erode(imageInit, imageErodee, Mat(), Point(-1,-1), 7,BORDER_CONSTANT, morphologyDefaultBorderValue() );
	dilate(imageErodee, imageDilatee, Mat(), Point(-1,-1), 2, BORDER_CONSTANT,morphologyDefaultBorderValue() );
	
	
	int nbCols=imageInit.cols;
	int nbRows=imageInit.rows;
	
	int pixel;
	Point PixelHaut;
	Point PixelBas;
	Point PixelGauche;
	Point PixelDroit;
	
	//! Point(x,y) is using (x,y) as (column,row)
	
	for(int i=0; i<nbRows;i++){
		for(int j=0; j<nbCols;j++){
			pixel=imageDilatee.at<uchar>(Point(j,i)); 
			if(pixel!=205) {
				PixelHaut=Point(j,i);
				i=nbRows;
				break;
			}
		}
	}
	
	for(int i=nbRows-10;i>0;i--){
		for(int j=nbCols; j>0;j--){
			pixel=imageDilatee.at<uchar>(Point(j,i)); 
			if(pixel!=205){
				PixelBas=Point(j,i);
				i=0;
				break;
			}
		}
	}
	
	for(int j=0;j<nbCols;j++){
		for(int i=0; i<nbRows;i++){
			pixel=imageDilatee.at<uchar>(Point(j,i)); 
			if(pixel!=205){
				PixelGauche=Point(j,i);
				j=nbCols;
				break;
			}
		}
	}
	
	for(int j=nbCols;j>0;j--){
		for(int i=nbRows-10; i>0;i--){
			pixel=imageDilatee.at<uchar>(Point(j,i)); 
			if(pixel!=205){
				PixelDroit=Point(j,i);
				j=0;
				break;
			}
		}
	}
	
	printf("PixelHaut : %d en x, %d en y\n", PixelHaut.x, PixelHaut.y);
	printf("PixelBas : %d en x, %d en y\n", PixelBas.x, PixelBas.y);
	printf("PixelGauche : %d en x, %d en y\n", PixelGauche.x, PixelGauche.y);
	printf("PixelDroit : %d en x, %d en y\n", PixelDroit.x, PixelDroit.y);
	
	//Resize l'image : rect ROI, puis transforme en mat encore
	int bordGauche =10;
	int bordHaut=10;
	int largeurImCropped=(PixelDroit.x-PixelGauche.x)+40;
	int hauteurImCropped=(PixelBas.y-PixelHaut.y)+20;
	Rect myROI(PixelGauche.x-bordGauche, PixelHaut.y-bordHaut, largeurImCropped, hauteurImCropped);
	imageFinale = imageDilatee(myROI);

	/*namedWindow( "Image amelioree", WINDOW_NORMAL);// Create a window for display.
    imshow( "Image amelioree", imageDilatee);
    waitKey(0);*/
    
    //On binarise la carte
    for(int i=0; i<hauteurImCropped;i++){
		for(int j=0; j<largeurImCropped;j++){
			pixel=imageFinale.at<uchar>(Point(j,i));
			if(pixel>150 && pixel<230){
				imageFinale.at<uchar>(Point(j,i))=0;
			}
		}
	}
    
	/*namedWindow( "Image finale", WINDOW_NORMAL);// Create a window for display.
    imshow( "Image finale", imageFinale);
    waitKey(0);   */                                       // Wait for a keystroke in the window
    
    imwrite("Image_finale.pgm", imageFinale);

	//ATTENTION ROS
	ros::init(argc, argv, "traitement_carte_node");
	ros::NodeHandle noeud_carte; // declaration noeud
	
	//Publish
	pub_point_0 = noeud_carte.advertise<geometry_msgs::Point>("/point_init",10);// creation d'un publisher pour le topic point_init
	pub_largeur_img = noeud_carte.advertise<std_msgs::Int32>("/nb_lignes",10);// creation d'un publisher pour le topic nb_lignes
	
	//On set le point à envoyer 
	ptAEnvoyer.x=(PixelGauche.x-bordGauche);
	ptAEnvoyer.y=(PixelHaut.y-bordHaut);
	
	nbLignes.data=nbRows;
	
	printf("Point à envoyer x: %f, y : %f \n", ptAEnvoyer.x,ptAEnvoyer.y);
	printf("Nombre de lignes de l'image envoyée : %d\n", nbLignes.data);

	ros::Rate loop_rate(100);

	while(ros::ok()){
		//ROS_INFO("\n\n\n******Envoi Point Initial************\n");
        pub_point_0.publish(ptAEnvoyer);
        pub_largeur_img.publish(nbLignes);
		ros::spinOnce();
		loop_rate.sleep();
	}	
    return 0;
	
}
