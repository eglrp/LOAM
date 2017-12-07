#include "odometry.h"
#include <math.h>
#include <iostream>
#include <fstream>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT>::Ptr CloudTypePtr;
using namespace std;

inline double calculate_cVal(vector<int>& S,const pcl::PointXYZRGB& centerPoint,pcl::PointCloud<PointT>::Ptr cloud);
inline double getCValue(int lo,int hi,PointT &centerPoint, CloudTypePtr cloud);
inline int setPoint2Red(pcl::PointXYZRGB& Point);
inline int setPoint2Green(pcl::PointXYZRGB& Point);
inline int setPointColor(PointT& Point,int r, int g, int b);
inline bool isPlanePoint(vector<double>& cVal,const pcl::PointXYZRGB& Point,pcl::KdTreeFLANN<PointT> kdtree);
inline bool isEdgePoint(vector<double>& cVal,const pcl::PointXYZRGB& Point,pcl::KdTreeFLANN<PointT> kdtree);
int read2PointCloud(std::string file2read,pcl::PointCloud<PointT>::Ptr cloud);
inline int findCIndex(std::vector<double>& cValue, double c);
inline void getPointDistanceSubregion(CloudTypePtr &cloud, vector<int>& S, PointT querypoint,vector<float>& distance);
inline int elimiAbnormalPoint(vector<int>& S,vector<float>& distance,pcl::PointCloud<PointT>::Ptr cloud);

int main ()
{
    pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    std::string dir = "../../data/Pk_select/";
    std::string filename = "1_1.XYZ";

    if(read2PointCloud((dir+filename),cloud)==-1)
    {
        printf("can not open file %s!\n",(dir+filename).c_str());
        return -1;
    }
    printf("Loaded %d data points from PCD\n",
            cloud->width * cloud->height);


    for(int i=0;i<cloud->width*cloud->height;++i)
    {
        cloud->points[i].r=255;
        cloud->points[i].g=255;
        cloud->points[i].b=255;
        cloud->points[i].a=128;
    }
    // creates kdtree object
    pcl::KdTreeFLANN<PointT> kdtree;
    // sets our randomly created cloud as the input
    kdtree.setInputCloud (cloud);
    //create a “searchPoint” which is assigned random coordinates
    PointT searchPoint;
    // K nearest neighbor search
    int cloudsize = cloud->width * cloud->height;
    int K = cloudsize>>7;
    printf("K:%d\n",K);
    std::vector<int> EdgePoint;
    std::vector<int> PlanarPoint;

    std::vector<int> pointIdxNKNSearch(K,0);
    std::vector<float> pointNKNSquaredDistance(K,0);
    std::vector<double> cVal(cloudsize,0);

   double c;
   for(int cnt=0;cnt < cloudsize;++cnt)
    {
        searchPoint=cloud->points[cnt];
        int lo = (cnt - K + cloudsize)%cloudsize;
        int hi = (cnt + K)%cloudsize;
        c = getCValue(lo,hi,searchPoint,cloud);
        cVal[cnt] = c;
    }
    printf("C!\n");
    for(int i=0;i<cloudsize;++i)
    {
        if(cVal[i]>0.03 && cVal[i]<0.08)  //0.01<c<0.08
        {
            setPoint2Red(cloud->points[i]);
            //printf("edge[%d]\n",i);
        }
        else if(cVal[i]<0.009)           //     c<0.005
              setPoint2Green(cloud->points[i]);
        if(cVal[i] > 1)
            setPointColor(cloud->points[i],255,255,0);
    }

    std::sort(cVal.begin(),cVal.end());
    for(int i=0;i<cVal.size();++i)
    {
        printf("%lf\n",cVal[i]);
    }
    printf("ready to show PointCloud\n");
    pcl::visualization::PCLVisualizer viewer;
    viewer.setCameraPosition(0,0,-3.0,0,-1,0);
    viewer.addCoordinateSystem(0.3);
    viewer.addPointCloud(cloud);

    while(!viewer.wasStopped())
    {
        viewer.spinOnce(100);
    }
    return (0);
}


inline double calculate_cVal(vector<int> &S,const pcl::PointXYZRGB & centerPoint,pcl::PointCloud<PointT>::Ptr cloud)
{
    double cVal;
    int Ssize=S.size();
    double D_centerPoint=sqrt((centerPoint.x)*(centerPoint.x)+(centerPoint.y)*(centerPoint.y)+(centerPoint.z)*(centerPoint.z));
    pcl::PointXYZRGB sumPoint(0,0,0);
    double D_sum=0;
    for(int i=0;i<Ssize;++i)
    {
        sumPoint.x+=(centerPoint.x-cloud->points[S[i]].x);
        sumPoint.y+=(centerPoint.y-cloud->points[S[i]].y);
        sumPoint.z+=(centerPoint.z-cloud->points[S[i]].z);
    }
    D_sum=sqrt((sumPoint.x)*(sumPoint.x)+(sumPoint.y)*(sumPoint.y)+(sumPoint.z)*(sumPoint.z));
    cVal=D_sum/(Ssize*D_centerPoint);
    return cVal;
}

inline double getCValue(int lo,int hi,PointT &centerPoint, CloudTypePtr cloud)
{
  double cValue;
  int cloudsize = cloud->width * cloud->height;
  if(lo < 0 || hi > cloudsize)
    return -1;
  double D_centerPoint=sqrt((centerPoint.x)*(centerPoint.x)+(centerPoint.y)*(centerPoint.y)+(centerPoint.z)*(centerPoint.z));
  pcl::PointXYZRGB sumPoint(0,0,0);
  double D_sum=0;
  for(int i=lo;i != hi; i = (i+1)%cloudsize)
  {
      sumPoint.x+=(centerPoint.x-cloud->points[i].x);
      sumPoint.y+=(centerPoint.y-cloud->points[i].y);
      sumPoint.z+=(centerPoint.z-cloud->points[i].z);
  }
  D_sum=sqrt((sumPoint.x)*(sumPoint.x)+(sumPoint.y)*(sumPoint.y)+(sumPoint.z)*(sumPoint.z));
  cValue = D_sum/((abs(hi - lo))*D_centerPoint);
  return cValue;
}

inline int findCIndex(std::vector<double>& cValue, double c)
{
    int size = cValue.size();
    for(int i = 0; i < size; ++i)
    {
      if(cValue[i] == c)
        return i;
    }
    return -1;
}

inline void getPointDistanceSubregion(CloudTypePtr &cloud, vector<int>& S, PointT querypoint,vector<float>& distance)
{
    PointT anotherPoint;
    float dx,dy,dz;
    int size = S.size();
    printf("Qurey(%f,%f,%f):\n",querypoint.x,querypoint.y,querypoint.z);
    for(int i = 0; i < size; ++i)
    {
        anotherPoint = cloud->points[S[i]];
        dx = querypoint.x - anotherPoint.x;
        dy = querypoint.y - anotherPoint.y;
        dz = querypoint.z - anotherPoint.z;
        distance[i] = sqrt(dx*dx + dy*dy + dz*dz);
        printf("%f\t",distance[i]);
    }
}

inline int setPoint2Red(pcl::PointXYZRGB& Point)
{
    Point.r=255;
    Point.g=0;
    Point.b=0;
    return 0;
}
inline int setPoint2Green(pcl::PointXYZRGB& Point)
{
    Point.r=0;
    Point.g=255;
    Point.b=0;
    return 0;
}

inline int setPointColor(PointT& Point,int r,int g, int b)
{
    Point.r = r;
    Point.g = g;
    Point.b = b;
    return 0;
}

inline bool isPlanePoint(vector<double>& cVal,const pcl::PointXYZRGB& Point,pcl::KdTreeFLANN<PointT> kdtree)
{
    return false;
}
inline bool isEdgePoint(vector<double>& cVal,const pcl::PointXYZRGB& Point,pcl::KdTreeFLANN<PointT> kdtree)
{
    return false;
}

int read2PointCloud(std::string file2read,pcl::PointCloud<PointT>::Ptr cloud)
{
    double tmp_d=0;
    pcl::PointXYZRGB point;
    int cnt=0;
    ifstream data_in(file2read.c_str());
    if(!data_in.is_open())
        return -1;
    while(data_in>>tmp_d)
    {
        if(cnt%3==0)
            point.x=tmp_d;
        else if(cnt%3==1)
            point.y=tmp_d;
        else if(cnt%3==2)
          {
            point.z=tmp_d;
            cloud->push_back(point);
          }
        ++cnt;
    }
    data_in.close();
    return 0;
}


inline int elimiAbnormalPoint(vector<int>& S,vector<float>& distance,pcl::PointCloud<PointT>::Ptr cloud)
{

    for(int i=0;i<distance.size();++i)
    {
        //printf("%2.8f\n",distance[i]);
      if(distance[i]>5)
        {
            S.erase(S.begin()+i);
            distance.erase(distance.begin()+i);
            --i;
        }
    }
    return 0;
}
