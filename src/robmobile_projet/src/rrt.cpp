#include <stdio.h>
#include <stdlib.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <iostream>
#include <sstream>
#include <fstream>
//ROS
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include "robmobile_projet/Tab_point.h"
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Int32.h>

//publisher
ros::Publisher pub_tab;
ros::Publisher pub_path_visu;

//Variable à envoyer
robmobile_projet::Tab_point tableauDePoint;

//booleen test position initiale recue
int pos_recue=0;

//Test pour savoir si clic
int cpt_clic;

//Stockage du point selectionné au clic
cv::Point pArrivee;

using namespace cv;
using namespace std;

//Ligne à compiler : g++ rrt.cpp -o rrt `pkg-config --cflags --libs opencv`


class RRT{
	public:
	//Constructeurs
    RRT(){
		RRT::rrtNode newNode;
		Point p;
		p.x=0;
		p.y=0;
		newNode.point=p;;
		newNode.parentId = 0;
		newNode.nodeId = 0;
		rrtTree.push_back(newNode);
	};
	
	RRT(Point input_p){
		RRT::rrtNode newNode;
		newNode.point=input_p;;
		newNode.parentId = 0;
		newNode.nodeId = 0;
		rrtTree.push_back(newNode);
	};    
	           
	struct rrtNode {
		int nodeId;
		int parentId;
		Point point;
		std::vector<rrtNode> children;
	};
	
	struct rrtArete{
		Point pPere;
		Point pFils;
	};
	
	//Attributs
    vector<rrtNode> rrtTree;
    vector<rrtArete> rrtListArete;
           
	//Declaration des fonctions
	vector<rrtNode> getTree();
    void setTree(vector<rrtNode> input_rrtTree);
    int getTreeSize();
    
    vector<rrtArete> getListArete();
    void setListArete(vector<rrtArete> input_rrtArete);
    int getListAreteSize();

    void addNewNode(rrtNode node);
    rrtNode removeNode(int nodeID);
    rrtNode getNode(int nodeID);
	Point getPoint(int nodeID);
	void setPoint(int nodeID, Point p);
	rrtNode setParentID(int parentID);
    rrtNode getParentID(int nodeID);
    void setParentID(int nodeID, int parentID);

	void addChildID(int nodeID, int childID);
    vector<rrtNode> getChildren(int nodeID);
    int getChildrenSize(int nodeID);

    int getNearestNodeID(Point p, int dmin);
    vector<int> getRootToEndPath(int endNodeID);

	float getDistanceEuclidienne(Point p1,Point p2);
};



vector<float> linspace(float start_in, float end_in, int num_in){

  vector<float> linspaced;

  float start = static_cast<float>(start_in);
  float end = static_cast<float>(end_in);
  double num = static_cast<float>(num_in);

  if (num == 0) { return linspaced; }
  if (num == 1) {
      linspaced.push_back(start);
      return linspaced;
    }
  double delta = (end - start) / (num - 1);
  for(int i=0; i < num-1; ++i){
      linspaced.push_back(start + delta * i);
  }
  linspaced.push_back(end); 
  return linspaced;
}

void generateTempPoint(RRT::rrtNode &tempNode,int maxX, int maxY)
{
    int xx = rand() % maxX + 1;
    int yy = rand() % maxY + 1;
    tempNode.point.x = xx;
    tempNode.point.y = yy;
}

bool checkIfInsideBoundary(RRT::rrtNode &tempNode,int maxX, int maxY)
{
    if(tempNode.point.x< 0 || tempNode.point.y < 0  || tempNode.point.x > maxX || tempNode.point.y > maxY ) {
		return false;
	}
    else {
		return true;
	}
}
bool isObstacle(Point p, Mat img){
	int pixel = img.at<uchar>(p);
	if(pixel==0){
		return true;
	}
	else{
		return false;
	}
}

bool checkIfOutsideObstacles(Mat img, Point tempNodePoint, Point nearestNodePoint, int sizeX, int sizeY){
	//on initialise une sorte de ligne entre nos deux pixels
	LineIterator it(img,tempNodePoint,nearestNodePoint,8);
	
	//boucle pour voir si il y a des obstacles sur cette ligne
	for(int i=0;i< it.count;i++,++it){
		if(isObstacle(it.pos(),img)){
			return true;
		}
	}
	return false;
}

bool addNewPointtoRRT(RRT &myRRT, RRT::rrtNode &tempNode, int rrtStepSize, Mat img,int sizeX,int sizeY,int dmin)
{
    int nearestNodeID = myRRT.getNearestNodeID(tempNode.point, dmin);

    RRT::rrtNode nearestNode = myRRT.getNode(nearestNodeID);

    double theta = atan2(tempNode.point.y - nearestNode.point.y,tempNode.point.x - nearestNode.point.x);

    tempNode.point.x = nearestNode.point.x + (rrtStepSize * cos(theta));
    tempNode.point.y = nearestNode.point.y + (rrtStepSize * sin(theta));

	
    if(checkIfInsideBoundary(tempNode,sizeX,sizeY) && !checkIfOutsideObstacles(img,tempNode.point,nearestNode.point,sizeX, sizeY))
    {
        tempNode.parentId = nearestNodeID;
        tempNode.nodeId = myRRT.getTreeSize();
        myRRT.addNewNode(tempNode);
        return true;
    }
    else
        return false;
}

void addBranchtoRRTTree(RRT::rrtNode &tempNode, RRT &myRRT)
{

	RRT::rrtNode parentNode = myRRT.getParentID(tempNode.nodeId);

	RRT::rrtArete areteTemp;
	areteTemp.pPere = parentNode.point;
	areteTemp.pFils = tempNode.point;
	myRRT.rrtListArete.push_back(areteTemp);

}

bool checkNodetoGoal(Point pGoal, RRT::rrtNode &tempNode)
{
    double distance = sqrt(pow(pGoal.x-tempNode.point.x,2)+pow(pGoal.y-tempNode.point.y,2));
    if(distance < 3)
    {
        return true;
    }
    return false;
}

void setFinalPathData(vector< vector<int> > &rrtPaths, RRT &myRRT, int i, vector<Point> &finalpath, Point pGoal)
{
    RRT::rrtNode tempNode;
    Point point;
    for(int j=0; j<rrtPaths[i].size();j++)
    {
        tempNode = myRRT.getNode(rrtPaths[i][j]);

        point.x = tempNode.point.x;
        point.y = tempNode.point.y;

        finalpath.push_back(point);
    }

    finalpath.push_back(pGoal);
}

/////////////////OPTIMISATION

vector<Point> CatmullRomCourbe(vector<Point> cheminOptimise, Mat imgColor, int taille){
		float t[5];
		t[0]=0;
		float alpha=0.5;
		float ti=0;
	
		for(int i=0;i<4;i++){
			t[i+1] = pow(pow(pow(cheminOptimise[i+1].x-cheminOptimise[i].x,2) + pow(cheminOptimise[i+1].y-cheminOptimise[i].y,2),0.5),alpha) + ti;
			ti=t[i+1];
		};
	
		/*for(int i=0;i<4;i++){
			printf("%f\n",t[i]);
		};*/

		vector<float> t2=linspace(t[1], t[2], taille);
		/*//test decoupage fonctionne
	
		for(int i=0;i<t2.size();i++){
			printf("%f\n",t2[i]);
		};*/

		vector<Point> A1;
		vector<Point> A2;
		vector<Point> A3;
		vector<Point> B1;
		vector<Point> B2;
		vector<Point> C ;

		for(int i=0;i<t2.size();i++){
	
			Point A1temp=(t[1]-t2[i])/(t[1]-t[0])*cheminOptimise[0] + (t2[i]-t[0])/(t[1]-t[0])*cheminOptimise[1];
			Point A2temp=(t[2]-t2[i])/(t[2]-t[1])*cheminOptimise[1] + (t2[i]-t[1])/(t[2]-t[1])*cheminOptimise[2];
			Point A3temp=(t[3]-t2[i])/(t[3]-t[2])*cheminOptimise[2] + (t2[i]-t[2])/(t[3]-t[2])*cheminOptimise[3];
			Point B1temp=(t[2]-t2[i])/(t[2]-t[0])*A1temp + (t2[i]-t[0])/(t[2]-t[0])*A2temp;
			Point B2temp=(t[3]-t2[i])/(t[3]-t[1])*A2temp + (t2[i]-t[1])/(t[3]-t[1])*A3temp;
			Point Ctemp =(t[2]-t2[i])/(t[2]-t[1])*B1temp + (t2[i]-t[1])/(t[2]-t[1])*B2temp;

			A1.push_back(A1temp);		
		  	A2.push_back(A2temp);
		  	A3.push_back(A3temp);
		  	B1.push_back(B1temp);
		  	B2.push_back(B2temp);
		  	C.push_back(Ctemp);

			//printf("%d,%d\n",A1temp.x,A1temp.y);
		};

		/*for(int i=1;i<t2.size()-1;i++){
			printf("%d,%d\n",C[i].x,C[i].y);
		};*/
	
		for(int i=0;i<C.size()-1;i++){
			   line(imgColor,C[i],C[i+1],Scalar(255,0,255),3,8);
		};
		/////////////
		///////////
		return C;
}

// Fonction callback appelée a chaque clic sur notre image donnée en param a setMouseCallback()
void on_mouse( int event, int x, int y, int, void*){
	 if( event != EVENT_LBUTTONDOWN ){
        return;
	}
    printf("Clic\n");
    Point pt;
    cpt_clic++;
    if(cpt_clic < 3){
      pt.x = x; 
      pt.y = y;
	}
    if(cpt_clic == 1){
        printf("\tPoint d'arrivée défini : x= %d, y=%d \n", pt.x, pt.y);
        pArrivee = pt;
    }
}


int main(int argc, char* argv[]){

// ********************** On récupère l'image à traiter ***************//
	Mat img;

	String imageName("/home/marion/catkin_ws/src/traitement_carte/src/Image_finale.pgm" );	//Chemin potentiellement changeant

	img = imread(imageName, CV_LOAD_IMAGE_GRAYSCALE);   // Read the file
	if(img.empty() ) // Check for invalid input
    {
        std::cout <<  "Could not open or find the image" << std::endl ;
        return -1;
    }

   
    //***********************On récupère x0_real_map,  y0_real_map, resolution ***********/  
    const char *pathToFileYaml("/home/marion/catkin_ws/src/robmobile_projet/maps/my_map_simu_gazebo.yaml");
	ifstream fichier(pathToFileYaml);
	vector<String> motVector;
	if(fichier){
    //L'ouverture s'est bien passée, on peut donc lire
    string motUseless; //Une variable pour stocker les mots lues
    
    while(fichier >> motUseless) //Tant qu'on n'est pas à la fin, on lit
    {
		motVector.push_back(motUseless);
      }
   }
   else
   {
      cout << "ERREUR: Impossible d'ouvrir le fichier en lecture." << endl;
   }
   
	//On stocke la résolution
	float resolution;
	resolution=strtof(motVector[3].c_str(),0);
	
	float x0_Real_Map,y0_Real_Map;
	String x0temp, y0temp;
	x0temp=motVector[5].c_str();
	x0temp=x0temp.substr(1,x0temp.size()-2);
	x0_Real_Map=strtof(x0temp.c_str(),0);
	y0temp=x0temp.substr(0,x0temp.size()-2);
	y0_Real_Map=strtof(y0temp.c_str(),0);
	
	//******* Initialisation de ROS *******//
    //Correspond au point d'origine du traitement carte (petite carte cropée)
    geometry_msgs::Point p1;
    //Nombre de lignes de l'image non traitée (vraie carte)
    int nbLignes; 
    //ROS ATTENTION
	ros::init(argc, argv, "rrt");
	ros::NodeHandle noeud_rrt; // declaration noeud
	//On attend 2 secondes
	usleep(2000000);
	
	//listener, ou plutot waitmsgs
	geometry_msgs::Point::ConstPtr ptInit_msgs = ros::topic::waitForMessage<geometry_msgs::Point>("/point_init");
	std_msgs::Int32::ConstPtr nbLignes_msgs = ros::topic::waitForMessage<std_msgs::Int32>("/nb_lignes");
	
	//on récupère le point p1
	p1.x=ptInit_msgs->x;
	p1.y=ptInit_msgs->y;
	printf("Point d'origine de la carte cropped en pixel, x : %f, y: %f\n",p1.x,p1.y);
	
	//On récupère le nombre de lignes de la carte
	nbLignes=nbLignes_msgs->data;
	printf("Largeur de la carte réelle en pixel: %d\n",nbLignes);
	
	//publish
	pub_tab = noeud_rrt.advertise<robmobile_projet::Tab_point>("/tab_chemin",10);
	pub_path_visu = noeud_rrt.advertise<nav_msgs::Path>("/path",10);
	
	ros::Rate loop_rate(10);
	
	//************** Récupation de la position initiale du robot dans la carte **************//
    
    int nbCols=img.cols;
	int nbRows=img.rows;
	
	int pixel;
	
	bool cheminTrouve=false;
	
	Point pDepart;
	//pDepart.x=400;
	//pDepart.y=240;

	double xrobot_recu, yrobot_recu;
	
	tf::TransformListener listener;
	while(ros::ok() && pos_recue==0){ //!pos_recue && 
		tf::StampedTransform transform;
		try{
			//listener.lookupTransform("/static_map","/base_link",ros::Time(0), transform);
			listener.lookupTransform("/map","/base_link",ros::Time(0), transform);
			printf("Pose initiale recue\n");
			xrobot_recu=transform.getOrigin().x();
			yrobot_recu=transform.getOrigin().y();
			pDepart.x=(int)((xrobot_recu-x0_Real_Map)/resolution-p1.x);
			//pDepart.y=(int)((4000-2000) - yrobot_recu/0.05 -1820); //A REFAIIIIIIRE
			//printf("nb lignes recu : %d, y0RealMap : %f, yrobot_recu :%f, resolution : %f, p1.y : %f\n \n",nbLignes,y0_Real_Map,yrobot_recu,resolution,p1.y);
			pDepart.y=(int)((nbLignes+y0_Real_Map/resolution) - yrobot_recu/resolution -p1.y); 
			pos_recue=1;
			break;
		}
		catch (tf::TransformException &ex) {
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
		}	
	}
	printf("Tf x: %f, y : %f\n",xrobot_recu,yrobot_recu);
	printf("Point de départ x: %d, y : %d\n",pDepart.x,pDepart.y);

	
	/*circle(img,pDepart,5,Scalar(255,0,0),2,1,0); //on affiche le pt de départ du robot
	namedWindow("Image originale", WINDOW_NORMAL);// Create a window for display.
    imshow("Image originale", img);                   // Show our image inside it.
    //gestion de la souris
	cpt_clic = 0; 
	setMouseCallback("Image originale", on_mouse,0);
    waitKey(5000);
    circle(img,pArrivee,5,Scalar(0,120,200),2,1,0); //on affiche le pt de départ du robot
	imshow("Image originale",img);                   // Show our image inside it.
	waitKey(2000);*/
	
	//On récupère point d'arrivée
	pArrivee.x=120;
	pArrivee.y=260;

	srand (time(NULL));
	
    //initializing rrtTree
    RRT rrt_courant(pDepart);
	
	//Definit le nombre de pixels sur une branche
    int rrtStepSize = 25;
    //Definit une distance minimale à laquelle le point généré ne sera pas conservé
    int dmin=20;

    vector< vector<int> > rrtPaths;
    vector<int> path;
    vector<Point> finalPath;
    int rrtPathLimit = 1;

    int shortestPathLength = 15;
    int shortestPath = -1;

    RRT::rrtNode tempNode;
    
    bool addNodeResult = false, nodeToGoal = false;
    
    while(!cheminTrouve){
		if(rrtPaths.size() < rrtPathLimit)
        {
			
            generateTempPoint(tempNode,nbCols,nbRows);
            //std::cout<<"tempnode generated"<<endl;
            addNodeResult = addNewPointtoRRT(rrt_courant,tempNode,rrtStepSize,img,nbCols,nbRows,dmin);
            if(addNodeResult)
            {
               // std::cout<<"tempnode accepted"<<endl;
                addBranchtoRRTTree(tempNode,rrt_courant);
               // std::cout<<"tempnode printed"<<endl;
                nodeToGoal = checkNodetoGoal(pArrivee,tempNode);
                if(nodeToGoal)
                {
                    path = rrt_courant.getRootToEndPath(tempNode.nodeId);
                    rrtPaths.push_back(path);
                    std::cout<<"New Path Found. Total paths "<<rrtPaths.size()<<endl;
                    //ros::Duration(10).sleep();
                    //std::cout<<"got Root Path"<<endl;
                }
            }
        }
        else //if(rrtPaths.size() >= rrtPathLimit)
        {
            cheminTrouve = true;
            std::cout<<"Finding Optimal Path"<<endl;
            for(int i=0; i<rrtPaths.size();i++)
            {
                if(rrtPaths[i].size() < shortestPath)
                {
                    shortestPath = i;
                    shortestPathLength = rrtPaths[i].size();
                }
            }
            setFinalPathData(rrtPaths, rrt_courant, shortestPath, finalPath, pArrivee);
            //rrt_publisher.publish(finalPath);
        }
	}
	
	Mat imgColor;
	cvtColor(img, imgColor, CV_GRAY2RGB);
	
	for(int i=0;i<rrt_courant.getListAreteSize();i++){
		line(imgColor,rrt_courant.rrtListArete[i].pPere,rrt_courant.rrtListArete[i].pFils,Scalar(0,255,0),1,8);
	}
	
	//On trace le graphe sur l'image
	for(int i=0;i<finalPath.size()-1;i++){
		   //line( img, start, end,Scalar( 0, 0, 0 ),thickness,lineType );
		   line(imgColor,finalPath[i],finalPath[i+1],Scalar(255,0,0),2,8);
	}
	
    //TESTConstruction chemin optimisé à partir du chemin
    Point pCourant=finalPath[0]; //initialisation du point courant
	vector<Point> cheminOptimise;
	cheminOptimise.push_back(pCourant);
	
	for(int i=1;i<=finalPath.size();i++){
		if(checkIfOutsideObstacles(img, pCourant, finalPath[i],nbCols,nbRows)){
			cheminOptimise.push_back(finalPath[i-1]);
			pCourant=finalPath[i-1];
		}
		/*if(rrt_courant.getDistanceEuclidienne(pCourant,pArrivee)<4){
		//if(pCourant.x==pArrivee.x && pCourant.y==pArrivee.y){
			i=finalPath.size();
		}*/
	}
	
	for(int i=0;i<cheminOptimise.size()-1;i++){
		   //line( img, start, end,Scalar( 0, 0, 0 ),thickness,lineType );
		   line(imgColor,cheminOptimise[i],cheminOptimise[i+1],Scalar(0,0,255),2,8);
   }
	


	////////////////////////////
//////////////////////////////////
///////////////////////////////////
/////////////////////////////////// OPTIMISATION

	//CAS entre le depart et le 1er noeud

	//taille correspondant au nombre de points choisis entre deux noeux (points extremes inclus)
	//int taille=15;
	int taille=3;
	float t0=0;
	float t1=pow(pow(pow(cheminOptimise[1].x-cheminOptimise[0].x,2) + pow(cheminOptimise[1].y-cheminOptimise[0].y,2),0.5),0.5);
	vector<float> t1_2_noeuds=linspace(t0, t1, taille);
	
	vector <Point> points;
	points.insert(points.begin(),pDepart);

	for(int i=1;i<t1_2_noeuds.size()+1;i++){
		points.push_back(Point(cheminOptimise[0].x+(cheminOptimise[1].x-cheminOptimise[0].x)*i/(taille),cheminOptimise[0].y+(cheminOptimise[1].y-cheminOptimise[0].y)*i/(taille)));
	};
	
	for(int i=0;i<t1_2_noeuds.size();i++){
		line(imgColor,points[i],points[i+1],Scalar(255,214,255),3,8);
	}
	//On a crée un vecteur noeud qui donne les n="taille" points entre le neoud de départ et le premier noeud

	//On réalise la strategie CatmullRom
	vector<Point> chemin_temp;
	vector< vector <Point> > Chemin_final_courbe;
	Chemin_final_courbe.push_back(points);
	vector<Point> bout_de_chemin_obtenu;
	
	for(int i=0;i<cheminOptimise.size()-2;i++){
		
		chemin_temp.push_back(cheminOptimise[i]);
		chemin_temp.push_back(cheminOptimise[i+1]);
		chemin_temp.push_back(cheminOptimise[i+2]);
		chemin_temp.push_back(cheminOptimise[i+3]);
		
		bout_de_chemin_obtenu=CatmullRomCourbe(chemin_temp, imgColor,taille);
		
		//on ajoute a chque fois entre deux nouveaux noueds, les n='taille' points crées
		Chemin_final_courbe.push_back(bout_de_chemin_obtenu);
		
		chemin_temp.clear();
			
	}
	/////////////////////////
//////////////////////////////////
////////////////////////////
	
	vector<Point> Final;
	Point Pfinal;
	//POUR OBTENIR LES POINTS DU CHEMIN A PARTIR DU PREMIER NOEUD!!!
	for(int i=0;i<cheminOptimise.size()-1;i++){
		for(int j=0;j<taille;j++){
			//printf("%d,%d\n",(Chemin_final_courbe[i])[j].x,(Chemin_final_courbe[i])[j].y);
			Pfinal.x=(Chemin_final_courbe[i])[j].x;
			Pfinal.y=(Chemin_final_courbe[i])[j].y;
			Final.push_back(Pfinal);
		}
	}
	
	for(int i=0;i<Final.size();i++){
		if((Final[i].x==Final[i+1].x)&&(Final[i].y==Final[i+1].y)){
			Final.erase(Final.begin()+i);
		}
	}
	
	///////AFFICHAGE
	circle(imgColor,pDepart,5,Scalar(138,43,226),2,1,0); //on affiche le pt de départ du robot
	circle(imgColor,pArrivee,5,Scalar(138,43,226),2,1,0); //on affiche le pt d'arrivée du robot
   	namedWindow( "Image avec chemin", WINDOW_AUTOSIZE);// Create a window for display.
    imshow( "Image avec chemin", imgColor);                   // Show our image inside it.
	waitKey(9000);
    
    	imwrite("Image avec chemin.pgm", imgColor);

	
	//MAP REELLE
	
	vector<float> floatX;
	vector<float> floatY;
	float ptX,ptY;
	
	//Remplissage du fram_id du path
	nav_msgs::Path path_a_envoyer;
	geometry_msgs::PoseStamped node_temp;
	path_a_envoyer.header.stamp=ros::Time::now();
	path_a_envoyer.header.frame_id="map";
	
	for(int i=0;i<Final.size();i++){
			printf("Point %d en pixel, x:%d, y:%d\n",i,Final[i].x,Final[i].y);
			//ptX=(float)( Final[i].x +p1.x )*resolution+x0_Real_Map;
			//ptY=(float)( Final[i].y -p1.y +nbRows/2)*resolution-y0_Real_Map-nbRows;
			ptX=(float)( Final[i].x +p1.x )*resolution+x0_Real_Map;
			//ptY=(float)( -Final[i].y -p1.y )*resolution-y0_Real_Map;
			ptY=((float)(nbLignes -Final[i].y -p1.y )*resolution+y0_Real_Map);
			printf("Point %d, en metres x: %f, y: %f\n",i,ptX,ptY);
			floatX.push_back(ptX);
			floatY.push_back(ptY);
			
			//Recuperation pour map visu path
			node_temp.header.stamp=ros::Time::now();
			node_temp.header.frame_id="map";
			node_temp.pose.position.x=ptX;
			node_temp.pose.position.y=ptY;
			node_temp.pose.position.z=0;
			node_temp.pose.orientation.x=1;
			node_temp.pose.orientation.y=0;
			node_temp.pose.orientation.z=0;
			node_temp.pose.orientation.w=0;
			
			path_a_envoyer.poses.push_back(node_temp);
			
	}
	
	

	//On remplit le tableau à envoyer
	for(int i=0;i<Final.size();i++){
		tableauDePoint.tab_points_X.push_back(floatX[i]);
		tableauDePoint.tab_points_Y.push_back(floatY[i]);
	}
	tableauDePoint.size=Final.size();	
	while(ros::ok()){

        pub_tab.publish(tableauDePoint);
        pub_path_visu.publish(path_a_envoyer);
		ros::spinOnce();
		//waitKey(5);
		loop_rate.sleep();
	}	

	return 0;

}

//Implémentation des parties de RRT
float RRT::getDistanceEuclidienne(Point p1,Point p2) {
        return std::sqrt(std::pow((p1.x - p2.x), 2) + std::pow((p1.y - p2.y), 2));
}

vector<RRT::rrtNode> RRT::getTree()
{
    return rrtTree;
}


void RRT::setTree(vector<RRT::rrtNode> input_rrtTree)
{
    rrtTree = input_rrtTree;
}

int RRT::getTreeSize()
{
    return rrtTree.size();
}

vector<RRT::rrtArete> RRT::getListArete()
{
    return rrtListArete;
}


void RRT::setListArete(vector<RRT::rrtArete> input_rrtArete)
{
    rrtListArete = input_rrtArete;
}

int RRT::getListAreteSize()
{
    return rrtListArete.size();
}

void RRT::addNewNode(RRT::rrtNode node)
{
    rrtTree.push_back(node);
}

/**
* removing a node from the RRT Tree
* @return the removed tree
*/
RRT::rrtNode RRT::removeNode(int id)
{
    RRT::rrtNode tempNode = rrtTree[id];
    rrtTree.erase(rrtTree.begin()+id);
    return tempNode;
}


RRT::rrtNode RRT::getNode(int id)
{
    return rrtTree[id];
}

/**
* return a node from the rrt tree nearest to the given point
* @param X position in X cordinate
* @param Y position in Y cordinate
* @return nodeID of the nearest Node
*/
int RRT::getNearestNodeID(Point p, int dmin)
{
    int i, returnID;
    double distance = 9999, tempDistance;
    for(i=0; i<this->getTreeSize(); i++)
    {
        tempDistance = getDistanceEuclidienne(p, getPoint(i));
        if (tempDistance < distance && tempDistance > dmin)
        {
            distance = tempDistance;
            returnID = i;
        }
    }
    return returnID;
}

Point RRT::getPoint(int nodeID){
	return rrtTree[nodeID].point;
}

void RRT::setPoint(int nodeID, Point p){
	rrtTree[nodeID].point=p;
}

/**
* returns parentID of the given node
*/
RRT::rrtNode RRT::getParentID(int id)
{
    return rrtTree[rrtTree[id].parentId];
}

/**
* set parentID of the given node
*/
void RRT::setParentID(int nodeID, int parentID)
{
    rrtTree[nodeID].parentId = parentID;
}

/**
* add a new childID to the children list of the given node
*/
void RRT::addChildID(int nodeID, int childID)
{
    rrtTree[nodeID].children.push_back(getNode(childID));
}

/**
* returns the children list of the given node
*/
vector<RRT::rrtNode> RRT::getChildren(int id)
{
    return rrtTree[id].children;
}

/**
* returns number of children of a given node
*/
int RRT::getChildrenSize(int nodeID)
{
    return rrtTree[nodeID].children.size();
}

/**
* returns path from root to end node
* @param endNodeID of the end node
* @return path containing ID of member nodes in the vector form
*/
vector<int> RRT::getRootToEndPath(int endNodeID)
{
    vector<int> path;
    path.push_back(endNodeID);
    while(rrtTree[path.front()].nodeId != 0)
    {
        path.insert(path.begin(),rrtTree[path.front()].parentId);
    }
    return path;
}
