#include "ros/ros.h"
#include "std_msgs/String.h"
#include <pcl/io/vtk_lib_io.h>
#include <sstream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/ply_io.h>
#include <math.h>
#include <pcl/surface/impl/convex_hull.hpp>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "pcl_ros/point_cloud.h"
#include <sensor_msgs/PointCloud2.h>
#include <iostream>

#include "SimplexSolver.h"
#include "exception.h"

#define PI 4*atan(1)

typedef std::vector<std::vector<int> > matrica;
typedef std::vector<uint32_t> vektor;

//PARAMETRI
std::string mesh = "spuzva_mt.ply";

double ugao_alfa = 1;
double ugao_beta = 5;
double raspon_hvataljki = 20;

double gustina = 1052; 				// PVC-ABS 								(kg/m3) 
double koeficijent_trenja = 0.65; 	// PVC-ABS                              abs = 0.46, guma = 0.84, srednje = 0.65
double masa = 0;					// podatak koji se dobije iz get_convex (kg)
double g = 9.81;					// gravitaciono ubrzanje 				(m/s2)
double F1 = 10, F2 = 10; 			// Sila grippera 						(N)


std::vector<float> dajNormalu(std::vector<uint32_t> idx, pcl::PointCloud<pcl::PointXYZ> pcl_cloud);
float dajD(std::vector<float> normala, std::vector<float> tacka);
bool daLiSuTrougloviSuprotni(std::vector<float> normala1, std::vector<float> normala2, std::vector<float> spoj_tezista, float alpha, float beta);
std::vector<float> dajTeziste(std::vector<uint32_t> idx1, pcl::PointCloud<pcl::PointXYZ> pcl_cloud);
std::vector<float> dajSpojTezista(std::vector<uint32_t> idx1, std::vector<uint32_t> idx2, pcl::PointCloud<pcl::PointXYZ> pcl_cloud);
double dajUdaljenostDoCentraMase(std::vector<uint32_t> idx, std::vector<float> centroid, pcl::PointCloud<pcl::PointXYZ> pcl_cloud);
double dajUgaoIzmedjuVektora(std::vector<float> v1, std::vector<float> v2);
std::vector<float> dajVektor(std::vector<float> v1, std::vector<float> v2);
double dajUdaljenostTacaka(std::vector<float> v1, std::vector<float> v2);
std::vector<float> dajVektorskiProizvod(std::vector<float> v1, std::vector<float> v2);

int main(int argc, char **argv)
{
	/*ros::init(argc, argv, "algorithm");
	ros::NodeHandle nh("~");
	std::string param;
	nh.getParam("/proba", param);
	std::cout << param << std::endl;*/

	pcl::PolygonMesh triangles;
	pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
	pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices());

	pcl::io::loadPolygonFilePLY(mesh, triangles);   
    pcl::fromPCLPointCloud2 (triangles.cloud, pcl_cloud);
	pcl::fromPCLPointCloud2 (triangles.cloud, *pcl_cloud_ptr);
	
	pcl::PointCloud<pcl::PointXYZ> input;
	if (pcl::io::loadPLYFile<pcl::PointXYZ> (mesh, input) == -1)
	{
		PCL_ERROR ("Could not read file. \n");
	}
	
	std::vector<float> sile;
	std::vector<float> normala1;
	std::vector<float> normala2;
	std::vector<float> tacka1;
	std::vector<float> tacka2;
	std::vector<int> par;
	matrica parovi;
	std::vector<std::vector<float> > centri_hvata;
	
	float D1;
	float D2;
	std::vector<bool> rezultat;
	int brojac = 0;
	
	
	std::vector<float> vektor_gravitacije;
	std::vector<float> koordinatni_pocetak;
	
	vektor_gravitacije.push_back(0);
	vektor_gravitacije.push_back(0);
	vektor_gravitacije.push_back(-1);
	
	koordinatni_pocetak.push_back(0);
	koordinatni_pocetak.push_back(0);
	koordinatni_pocetak.push_back(0);

	std_msgs::String msg;
	std::stringstream ss;
	
	// PRORACUN PAROVA TROUGLOVA
	for(int i = 0; i < triangles.polygons.size() - 1; i++) {

		normala1 = dajNormalu(triangles.polygons[i].vertices, pcl_cloud);
		tacka1.push_back(pcl_cloud.points[triangles.polygons[i].vertices[0]].data[0]);
		tacka1.push_back(pcl_cloud.points[triangles.polygons[i].vertices[0]].data[1]);
		tacka1.push_back(pcl_cloud.points[triangles.polygons[i].vertices[0]].data[2]);	

		for(int j = i + 1; j < triangles.polygons.size(); j++) {
			
			std::vector<float> spoj_tezista;
			spoj_tezista = dajSpojTezista(triangles.polygons[i].vertices, triangles.polygons[j].vertices, pcl_cloud);
			
			normala2 = dajNormalu(triangles.polygons[j].vertices, pcl_cloud);
			tacka2.push_back(pcl_cloud.points[triangles.polygons[j].vertices[0]].data[0]);
			tacka2.push_back(pcl_cloud.points[triangles.polygons[j].vertices[0]].data[1]);
			tacka2.push_back(pcl_cloud.points[triangles.polygons[j].vertices[0]].data[2]);			
			tacka2.clear();
			
			if(daLiSuTrougloviSuprotni(normala1, normala2, spoj_tezista, ugao_alfa, ugao_beta)) {
				par.push_back(i);
				par.push_back(j);
				
				parovi.push_back(par);
				par.clear();
			}
		}
		
		tacka1.clear();
			
	}

	//CENTAR MASE
	Eigen::Vector4f centroid;
	std::vector<float> centroid_vector;
	pcl::ConvexHull<pcl::PointXYZ> chull;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
	chull.setInputCloud(input.makeShared());
	chull.setDimension(3);
	chull.reconstruct (*cloud_hull);
	pcl::compute3DCentroid(*cloud_hull, centroid);
	
	centroid_vector.push_back(centroid[0]);
	centroid_vector.push_back(centroid[1]);
	centroid_vector.push_back(centroid[2]);

	//std::cout << centroid[0] << " " << centroid[1] << " " << centroid[2] << std::endl;

	// PLANE CUT, RACUNANJE SILA I MOMENATA
	for(int i = 0; i < parovi.size(); i++) {

		
		
		for(int j = 0; j < input.size(); j++) {
			
			// Izdvajanje tacki ispod ravni
			if(0*(input.points[parovi[i][0]].x - input.points[j].x) + 0*(input.points[parovi[i][0]].y - input.points[j].y) + 1*(input.points[parovi[i][0]].z - 				input.points[j].z) > 0)
			{
				inliers_plane->indices.push_back(j);
			}
		}
		
		// Formiranje objekta presjecenog sa ravni
		pcl::PointCloud<pcl::PointXYZ> temp_cloud = input;
		extract.setInputCloud(temp_cloud.makeShared());
  		extract.setIndices(inliers_plane);
  		extract.filter(temp_cloud);
		inliers_plane->indices.clear();
		

		// Proracun convex hull-a i volumena objekta
		pcl::ConvexHull<pcl::PointXYZ> chull;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
		chull.setInputCloud(temp_cloud.makeShared());
		chull.setDimension(3);
		chull.setComputeAreaVolume(true);
		chull.reconstruct (*cloud_hull);	
		
		
		// Proracun potrebnih vektora
		std::vector<float> norm_prvi = dajNormalu(triangles.polygons[parovi[i][0]].vertices, pcl_cloud);
		std::vector<float> norm_drugi = dajNormalu(triangles.polygons[parovi[i][1]].vertices, pcl_cloud);
		std::vector<float> dv_1 = dajVektor(dajTeziste(triangles.polygons[parovi[i][0]].vertices, pcl_cloud), centroid_vector);
		std::vector<float> dv_2 = dajVektor(dajTeziste(triangles.polygons[parovi[i][1]].vertices, pcl_cloud), centroid_vector);
		std::vector<float> teziste1 = dajTeziste(triangles.polygons[parovi[i][0]].vertices, pcl_cloud);
		std::vector<float> teziste2 = dajTeziste(triangles.polygons[parovi[i][1]].vertices, pcl_cloud);
		std::vector<float> spoj = dajVektor(teziste1, teziste2);
		std::vector<float> stranica1 = dajVektor(teziste1, centroid_vector);
		std::vector<float> stranica2 = dajVektor(teziste2, centroid_vector);
		std::vector<float> vektorski1;
		std::vector<float> vektorski2;
		std::vector<float> uglovi_graspa;
		std::vector<float> centar_hvata;
		
		
		// Inicijalizacija varijabli
		double alfa_1, alfa_2, beta_1, beta_2;
		//double d1 = dajUdaljenostDoCentraMase(triangles.polygons[parovi[i][0]].vertices, centroid_vector, pcl_cloud);
		//double d2 = dajUdaljenostDoCentraMase(triangles.polygons[parovi[i][1]].vertices, centroid_vector, pcl_cloud);
		double G;		
		double d1_x, d1_y, d2_x, d2_y;
		double Ftr_1x, Ftr_2x, Ftr_1y, Ftr_2y;
		double F1_x, F2_x, F1_y, F2_y;
		double Mtr_1x, Mtr_2x, Mtr_1y, Mtr_2y;
		double M1_x, M2_x, M1_y, M2_y;
		double brojnik, nazivnik;
		double G_krak, d1_krak, d2_krak;
		
		
		// Proracun ravni i kraka sile teze za moment
		std::vector<float> treca_tacka;
		std::vector<float> normala_krak;
		std::vector<float> ab;
		std::vector<float> ac;
		treca_tacka.push_back(teziste1[0]);
		treca_tacka.push_back(teziste1[1]);
		treca_tacka.push_back(teziste1[2]+1);
		ab.push_back(treca_tacka[0] - teziste1[0]);
		ab.push_back(treca_tacka[1] - teziste1[1]);
		ab.push_back(treca_tacka[2] - teziste1[2]);
		ac.push_back(treca_tacka[0] - teziste2[0]);
		ac.push_back(treca_tacka[1] - teziste2[1]);
		ac.push_back(treca_tacka[2] - teziste2[2]);
		normala_krak.push_back(ab[1]*ac[2] - ab[2]*ac[1]);
		normala_krak.push_back(-(ab[0]*ac[2] - ab[2]*ac[0]));
		normala_krak.push_back(ab[0]*ac[1] - ab[1]*ac[0]);
		brojnik = normala_krak[0]*centroid_vector[0]+normala_krak[1]*centroid_vector[1]+normala_krak[2]*centroid_vector[2] + dajD(normala_krak, teziste1);
		G_krak = brojnik/sqrt(pow(normala_krak[0], 2) + pow(normala_krak[1], 2) + pow(normala_krak[2], 2));
		

		// Proracun ravni i kraka sila hvata za moment
		vektorski1 = dajVektorskiProizvod(stranica1, normala_krak);
		brojnik = sqrt(pow(vektorski1[0], 2) + pow(vektorski1[1], 2) + pow(vektorski1[2], 2));
		nazivnik = sqrt(pow(normala_krak[0], 2) + pow(normala_krak[1], 2) + pow(normala_krak[2], 2));
		d1_krak = brojnik/nazivnik;
		
		vektorski2 = dajVektorskiProizvod(stranica2, normala_krak);
		brojnik = sqrt(pow(vektorski2[0], 2) + pow(vektorski2[1], 2) + pow(vektorski2[2], 2));
		d2_krak = brojnik/nazivnik;
		
		
		// Proracun sila i momenata
		masa = chull.getTotalVolume()*gustina;		
		
		alfa_1 = fabs(PI/2 - dajUgaoIzmedjuVektora(norm_prvi, vektor_gravitacije));
		alfa_2 = fabs(PI/2 - dajUgaoIzmedjuVektora(norm_drugi, vektor_gravitacije));
		beta_1 = fabs(PI/2 - dajUgaoIzmedjuVektora(dv_1, vektor_gravitacije));
		beta_2 = fabs(PI/2 - dajUgaoIzmedjuVektora(dv_2, vektor_gravitacije));
		
		G = masa*g;		

		d1_x = cos(beta_1) * d1_krak;
		d1_y = sin(beta_1) * d1_krak;
		d2_x = cos(beta_2) * d2_krak;
		d2_y = sin(beta_2) * d2_krak;
		
		Ftr_1x = sin(alfa_1) * koeficijent_trenja * G;
		Ftr_2x = sin(alfa_2) * koeficijent_trenja * G;				
		Ftr_1y = cos(alfa_1) * koeficijent_trenja * G;
		Ftr_2y = cos(alfa_2) * koeficijent_trenja * G;

		F1_x = cos(alfa_1) * F1;
		F2_x = cos(alfa_2) * F2;
		F1_y = sin(alfa_1) * F1;
		F2_y = sin(alfa_2) * F2;
		
		Mtr_1x = Ftr_1x * d1_y;
		Mtr_2x = Ftr_2x * d2_x;
		Mtr_1y = Ftr_1y * d1_y;
		Mtr_2y = Ftr_2y * d2_x;

		M1_x = F1_x * d1_y;
		M2_x = F2_x * d2_x;
		M1_y = F1_y * d1_y;
		M2_y = F2_y * d2_x;
		
		std::cout<< "Sile: " << Ftr_1y + Ftr_2y - G << std::endl;
		std::cout<< "Moment hvata: " << Mtr_1x - Mtr_2x + Mtr_1y - Mtr_2y + M1_x - M2_x - M1_y + M2_y << std::endl;
		std::cout<< "Moment sile teze: " << G_krak*G << std::endl; // uvijek djeluje po osi koja je drugacija od ose momenta hvata
		std::cout<< "Sredina graspa - x: " <<(teziste1[0] + teziste2[0])/2<< " y: " <<(teziste1[1] + teziste2[1])/2<<" z: " <<(teziste1[2] + teziste2[2])/2 <<std::endl<< std::endl;

	}
	
	// FORMIRANJE .PLY MODELA OD PAROVA TROUGLOVA
	for(int k=0; k < parovi.size(); k++) {
		inliers->indices.push_back(triangles.polygons[parovi[k][0]].vertices[0]);
		inliers->indices.push_back(triangles.polygons[parovi[k][0]].vertices[1]);
		inliers->indices.push_back(triangles.polygons[parovi[k][0]].vertices[2]);
		inliers->indices.push_back(triangles.polygons[parovi[k][1]].vertices[0]);
		inliers->indices.push_back(triangles.polygons[parovi[k][1]].vertices[1]);
		inliers->indices.push_back(triangles.polygons[parovi[k][1]].vertices[2]);
	}
	extract.setInputCloud(pcl_cloud_ptr);
  	extract.setIndices(inliers);
  	//extract.setNegative(true);
  	extract.filter(*pcl_cloud_ptr);
	pcl::io::savePLYFileASCII ("no_triangle.ply", *pcl_cloud_ptr); 
	

	return 0;
}

// Funkcija za racunanje normale. Normala se koristi radi stepena paralelnosti.
std::vector<float> dajNormalu(std::vector<uint32_t> idx, pcl::PointCloud<pcl::PointXYZ> pcl_cloud) {

	std::vector<float> ab;
	std::vector<float> ac;
	std::vector<float> normala;
	
	ab.push_back(pcl_cloud.points[idx[1]].data[0] - pcl_cloud.points[idx[0]].data[0]);
	ab.push_back(pcl_cloud.points[idx[1]].data[1] - pcl_cloud.points[idx[0]].data[1]);
	ab.push_back(pcl_cloud.points[idx[1]].data[2] - pcl_cloud.points[idx[0]].data[2]);
	
	ac.push_back(pcl_cloud.points[idx[2]].data[0] - pcl_cloud.points[idx[0]].data[0]);
	ac.push_back(pcl_cloud.points[idx[2]].data[1] - pcl_cloud.points[idx[0]].data[1]);
	ac.push_back(pcl_cloud.points[idx[2]].data[2] - pcl_cloud.points[idx[0]].data[2]);

	normala.push_back(ab[1]*ac[2] - ab[2]*ac[1]);
	normala.push_back(-(ab[0]*ac[2] - ab[2]*ac[0]));
	normala.push_back(ab[0]*ac[1] - ab[1]*ac[0]);
	
	
	return normala;
}


// Funkcija za racunanje D komponente trougla(ravni). Koristit ce se kao stepen poklapanja.
float dajD(std::vector<float> normala, std::vector<float> tacka) {
	
	float D;

	return D = - normala[0]*tacka[0] - normala[1]*tacka[1] - normala[2]*tacka[2];
}

// Pronalazak antipodalnih trouglova
bool daLiSuTrougloviSuprotni(std::vector<float> normala1, std::vector<float> normala2, std::vector<float> spoj_tezista, float alpha, float beta) {
	
	float izraz1, izraz2, K1, K2, udaljenost;

	K1 = sqrt(pow(normala1[0],2)+pow(normala1[1],2)+pow(normala1[2],2))*sqrt(pow(normala2[0],2)+pow(normala2[1],2)+pow(normala2[2],2));
	izraz1 = (normala1[0]*normala2[0] + normala1[1]*normala2[1] + normala1[2]*normala2[2])/K1;

	K2 = sqrt(pow(normala1[0],2)+pow(normala1[1],2)+pow(normala1[2],2))*sqrt(pow(spoj_tezista[0],2)+pow(spoj_tezista[1],2)+pow(spoj_tezista[2],2));
	izraz2 = (normala1[0]*spoj_tezista[0] + normala1[1]*spoj_tezista[1] + normala1[2]*spoj_tezista[2])/K2;
	
	udaljenost = sqrt(pow(spoj_tezista[0], 2) + pow(spoj_tezista[1], 2) + pow(spoj_tezista[2], 2));	
	
	if(izraz1 >= cos(alpha*PI/180) && izraz2 >= cos(beta*PI/180) && udaljenost <= raspon_hvataljki) {
			
				
		return true;
	}
		
	return false;
}

std::vector<float> dajTeziste(std::vector<uint32_t> idx1, pcl::PointCloud<pcl::PointXYZ> pcl_cloud) {

	std::vector<float> teziste;

	teziste.push_back((pcl_cloud.points[idx1[0]].data[0] + pcl_cloud.points[idx1[1]].data[0] + pcl_cloud.points[idx1[2]].data[0])/3);
	teziste.push_back((pcl_cloud.points[idx1[0]].data[1] + pcl_cloud.points[idx1[1]].data[1] + pcl_cloud.points[idx1[2]].data[1])/3);
	teziste.push_back((pcl_cloud.points[idx1[0]].data[2] + pcl_cloud.points[idx1[1]].data[2] + pcl_cloud.points[idx1[2]].data[2])/3);

	return teziste;

}

std::vector<float> dajSpojTezista(std::vector<uint32_t> idx1, std::vector<uint32_t> idx2, pcl::PointCloud<pcl::PointXYZ> pcl_cloud) {

	std::vector<float> teziste1, teziste2, spoj_tezista;
	
	teziste1 = dajTeziste(idx1, pcl_cloud);
	teziste2 = dajTeziste(idx2, pcl_cloud);

	spoj_tezista.push_back(teziste1[0]-teziste2[0]);
	spoj_tezista.push_back(teziste1[1]-teziste2[1]);
	spoj_tezista.push_back(teziste1[2]-teziste2[2]);

	return spoj_tezista;
	
}

double dajUdaljenostDoCentraMase(std::vector<uint32_t> idx, std::vector<float> centroid, pcl::PointCloud<pcl::PointXYZ> pcl_cloud) {

	double udaljenost(0);
	std::vector<float> teziste;
	
	teziste = dajTeziste(idx, pcl_cloud);
	
	udaljenost = sqrt(pow(teziste[0]-centroid[0], 2) + pow(teziste[1]-centroid[1], 2) + pow(teziste[2]-centroid[2], 2));
	
	return udaljenost;
}

double dajUgaoIzmedjuVektora(std::vector<float> v1, std::vector<float> v2) {
	
	float K, ugao;	

	K = sqrt(pow(v1[0],2)+pow(v1[1],2)+pow(v1[2],2))*sqrt(pow(v2[0],2)+pow(v2[1],2)+pow(v2[2],2));
	ugao = (v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2])/K;
	
	return acos(ugao);
}

std::vector<float> dajVektor(std::vector<float> v1, std::vector<float> v2) {
	
	std::vector<float> v;
	v.push_back(v2[0]-v1[0]);
	v.push_back(v2[1]-v1[1]);
	v.push_back(v2[2]-v1[2]);
	
	return v;
}

double dajUdaljenostTacaka(std::vector<float> v1, std::vector<float> v2) {
	
	double udaljenost = 0;
	udaljenost = sqrt(pow(v1[0]-v2[0], 2) + pow(v1[1]-v2[1], 2) + pow(v1[2]-v2[2], 2));

	return udaljenost;
}

std::vector<float> dajVektorskiProizvod(std::vector<float> v1, std::vector<float> v2) {
	std::vector<float> vektorski;

	vektorski.push_back(v1[1]*v2[2] - v1[2]*v2[1]);
	vektorski.push_back(-(v1[0]*v2[2] - v1[2]*v2[0]));
	vektorski.push_back(v1[0]*v2[1] - v1[1]*v2[0]);

	return vektorski;
}




















