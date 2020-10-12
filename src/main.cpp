#include "environment/robot.hpp"
#include <cstdio>
#include <opencv2/opencv.hpp>
#include <experimental/random>
// #include "environment/ocm.hpp"
#include <iterator>
#include <geometric_shapes/shapes.h>

#include <cstring>
#include <string>

#include "pool.hpp"

using namespace std;

using namespace cv;

// void MyLine( Mat img, Point start, Point end );


// struct genome
// {
//   shared_ptr<cv::Mat> map;
//   vector<Point> waypoints;
//   float fitness = 0;
// };


// Point randomPointShift(Point p, int magnitude = 200){

//   Point p2(p);

//   int shift_x = std::experimental::randint(-magnitude, magnitude);
//   int shift_y = std::experimental::randint(-magnitude, magnitude);
//   p2.x +=  shift_x;
//   p2.y +=  shift_y;
//   return p2;

// }

// vector<Point> genWaypoints(int width, int height, Point start, int n = 10){
//   // Generate list with valid waypoints
//   // For now the only rule is that the points need to be placed on the map
//   vector<Point> waypoints;
//   waypoints.push_back(start);
//   Point shift_p(start);
//   for (int i = 0; i < n; ++i) {
//     bool valid = false;

//     while(true){
//       Point tmp = randomPointShift(shift_p);
//       if((tmp.x < width) && (tmp.y < height) && (tmp.x > 0) && tmp.y > 0){
// 	shift_p = tmp;
// 	waypoints.push_back(tmp);
// 	break;
//       }
//     }
//   }
//   return waypoints;
// }

// void printFitness(vector<genome> gens){
//   for(auto j : gens){
//     cout << j.fitness << " ";
//     break;
//   }
//   cout << endl;
// }


// class GenPool{
//   // Generate population
//   // Perform crossover
//   // perform mutation
//   // perform selection

// public:
//   GenPool(int width, int height, Point start, Point end):width(width), height(height), start(start), end(end) {}
//   void populatePool(int size, int waypoints);

//   // Returns currently best fitness
//   float update(int iterations = 100);

// // private:
//   vector<genome> gens;
//   const int width;
//   const int height;
//   const Point start;
//   const Point end;

//   void crossover();
//   void mutation();
//   void selection();
//   float calFittness(struct genome &gen);
//   float calOcc(struct genome &gen);
//   float calTime(struct genome &gen, int speed = 5);
//   struct genome getBest();
//   void conditionalPointShift(Point &p, int magnitude = 5);
//   void randomInsert(struct genome &gen);
//   void randomRemove(struct genome &gen);
// };

// bool compareFitness(const struct genome &genA, const struct genome &genB){
//   return genA.fitness > genB.fitness;
// }

// struct genome GenPool::getBest(){
//   sort(gens.begin(), gens.end(), compareFitness);
//   return gens[0];
// }

// void GenPool::populatePool(int size, int waypoints)
// {
//   // Create initial population of given Size

//   for (int i=0; i<size; ++i) {
//     struct genome gen;
//     gen.map = make_shared<Mat>(Mat(height, width, CV_8U, Scalar(0)));
//     gen.waypoints = genWaypoints(width, height, start, waypoints);
//     gen.waypoints.push_back(end);

//     gen.fitness = this->calFittness(gen);
//     this->gens.push_back(gen);
//   }
//   printFitness(gens);
// }

// float GenPool::calFittness(struct genome &gen)
// {
//   // Maximize occ minimize time
//   return calOcc(gen)/100 - calTime(gen)*100;
// }

// float GenPool::calOcc(struct genome &gen)
// {

//   *gen.map = Scalar(0);
//   // cout << "Initial cost: " << cv::sum(map) << endl;

//   auto iter = gen.waypoints.begin();
//   Point current = *iter;
//   iter++;
//   do{
//     MyLine(*gen.map, current, *iter);
//     current = *iter;
//     // cout << current << endl;
//     iter++;
//   } while(iter != gen.waypoints.end());
//   return cv::sum(*gen.map)[0] ; // / (width * height);
// }


// float GenPool::calTime(struct genome &gen, int speed)
// {

//   float dist = 0;
//   auto iter = gen.waypoints.begin();
//   Point current = *iter;
//   iter++;
//   do{
//     // MyLine(map, current, *iter);
//     // Sum up all distances the roboter needs to travel
//     dist += cv::norm(current - *iter);
//     current = *iter;
//     // cout << current << endl;
//     iter++;
//   } while(iter != gen.waypoints.end());

//   return dist / speed;

// }

// float GenPool::update(int iterations){
//   for (int i = 0; i <= iterations; ++i) {
//     // cout << "Crossover" << endl;
//     crossover();
//     // cout << "Mutation" << endl;
//     mutation();
//     // cout << "CSelection" << endl;
//     selection();
//     if(i % 1000 == 0){
//       cout << ""
//       cv::putText(*gens.at(0).map,"it=" + std::to_string(i) + "Nodes="+std::to_string(gens.at(0).waypoints.size()), Point(10,900), CV_FONT_HERSHEY_SIMPLEX, 1, 255);
//       cv::imwrite("res/it_" + std::to_string(i) + ".jpg", *gens.at(0).map);
//     }
//   }
//   return 42.0;
// }

// // Return elements of vec between start and stop index
// template<typename T> vector<T> slice(vector<T>& vec, int start_idx, int stop_idx){

//   return vector<T>(vec.begin() + start_idx, vec.begin() + stop_idx);
// }

// //Slice vec and delete sliced elements
// template<typename T> vector<T> sliceErase(vector<T>& vec, int start_idx, int stop_idx){
//   auto result = slice<T>(vec, start_idx, stop_idx);
//   vec.erase(vec.begin() + start_idx, vec.begin() + stop_idx);
//   return result;
// }

// // Append content of vecB to vecA
// template<typename T> void joinSlices(vector<T>& vecA, vector<T>& vecB){
//   vecA.reserve(vecA.size() + vecB.size());
//   vecA.insert(vecA.end(), vecB.begin(), vecB.end());
// }

// void printWaypoints(vector<Point>& waypoints){
//   cout << "Waypoints: " << endl;
//   for(const auto i : waypoints){
//     cout << i << endl;
//   }
// }


// void GenPool::crossover()
// {
//   // get best two individuals
//   // Assume that gens are already sorted according to their finess
//   vector<Point> parent1 = gens[0].waypoints;
//   vector<Point> parent2 = gens[1].waypoints;

//   auto slice1 = sliceErase(parent1, (int) parent1.size()/2, parent1.size());
//   auto slice2 = sliceErase(parent2, (int) parent2.size()/2, parent2.size());

//   joinSlices(parent1, slice2);
//   joinSlices(parent2, slice1);

//   gens[gens.size()-2].waypoints.assign(parent1.begin(), parent1.end());
//   gens[gens.size()-1].waypoints.assign(parent2.begin(), parent2.end());

// }

// void GenPool::conditionalPointShift(Point &p, int magnitude){

//   while(true){
//     int shift_x = std::experimental::randint(-magnitude, magnitude);
//     int shift_y = std::experimental::randint(-magnitude, magnitude);
//     if(((p.x + shift_x) > 0)
//        && ((p.x + shift_x) < width)
//        && ((p.y + shift_y) > 0)
//        && ((p.y + shift_y) < height)){
//       p.x +=  shift_x;
//       p.y +=  shift_y;
//       break;
//     }
//   }
// }

// void GenPool::randomInsert(struct genome &gen){
//   int node = std::experimental::randint(1, (int) gen.waypoints.size()-1);
//   Point p(gen.waypoints.at(node));

//   conditionalPointShift(p);
//   // cout << "Try shift" << endl;
//   auto beg = gen.waypoints.begin();
//   // cout << "Size before: " << gen.waypoints.size() << endl;
//   gen.waypoints.insert(beg + node, p);
//   // cout << "Size after: " << gen.waypoints.size() << endl;
//   // cout << "Inserted!" << endl;
// }

// void GenPool::randomRemove(struct genome &gen){
//   int node = std::experimental::randint(1, (int) gen.waypoints.size()-1);

//   auto beg = gen.waypoints.begin();
//   gen.waypoints.erase(beg + node);
//   // cout << "Inserted!" << endl;
// }


// void GenPool::mutation(){
//   for(int i=0; i<gens.size();i++){
//     // randomInsert(gens[i]);
//     if (i == 0) {
//       randomInsert(gens[i]);
//     } else {
//       int node = std::experimental::randint(1, (int) gens[i].waypoints.size()-1);
//       randomRemove(gens[i]);
//       conditionalPointShift(gens[i].waypoints[node], 50);
//       // cout << "Test" << endl;
//     }
//     // gens[i].fitness = calFittness(gens[i]);
//   }
// }

// void GenPool::selection(){
//   for (int i=0; i<gens.size(); ++i) {
//     gens[i].fitness = calFittness(gens[i]);
//   }

//   sort(gens.begin(), gens.end(), compareFitness);
//   printFitness(gens);
// }


int main(int argc, char *argv[])
{
  Robot rob;
  // foo::OccupancyMap ocm(10, 10);
  // cout << "Mat = " << endl << *ocm.get_ocm() << endl;

  const uint32_t width = 1000;
  const uint32_t height = 1000;
  const Point start(0,50);
  const Point stop(999, 500);

  uint8_t im[height][width] =  { 0 };
  Mat map(height, width, CV_8U, im);

  vector<Point> waypoints = opti_ga::genWaypoints(width, height, start);

  // auto occ_fit = cal_occ(waypoints, width, height);
  // auto time_fit = cal_time(waypoints, 3);



  opti_ga::GenPool pool(width, height, start, stop);
  pool.populatePool(5, 50);
  // cout << "Fitness: " << pool.getBest().fitness << endl;
  // for(const auto i : pool.gens){
  //   cout << i.fitness << endl;
  // }
  pool.update(100000);
  // imwrite("res/it_" + std::to_string(), *pool.getBest().map);
  // imshow("Res: ", *pool.getBest().map);
  // waitKey(0);


  // Optimization stage 1:
  // TODO: Waypoint generation
  // TODO: Geometric rotation and transformation on map

  // First assumption to ease the constraints is to get the robot a round shape




  // printf("Hello Rob: %d\n", rob.hello());
  return 0;
}

// void MyLine( Mat img, Point start, Point end )
// {
//   int thickness = 4;
//   int lineType = LINE_4;
//   line( img,
//     start,
//     end,
//     Scalar( 255 ),
//     thickness,
//     lineType );
// }
