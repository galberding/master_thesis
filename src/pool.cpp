#include "pool.hpp"

using namespace opti_ga;


//-----------------------------------------
Point opti_ga::randomPointShift(Point p, int magnitude){

  Point p2(p);

  int shift_x = std::experimental::randint(-magnitude, magnitude);
  int shift_y = std::experimental::randint(-magnitude, magnitude);
  p2.x +=  shift_x;
  p2.y +=  shift_y;
  return p2;

}


vector<Point> opti_ga::genWaypoints(int width, int height, Point start, int n){
  // Generate list with valid waypoints
  // For now the only rule is that the points need to be placed on the map
  vector<Point> waypoints;
  waypoints.push_back(start);
  Point shift_p(start);
  for (int i = 0; i < n; ++i) {
    bool valid = false;

    while(true){
      Point tmp = randomPointShift(shift_p);
      if((tmp.x < width) && (tmp.y < height) && (tmp.x > 0) && tmp.y > 0){
	shift_p = tmp;
	waypoints.push_back(tmp);
	break;
      }
    }
  }
  return waypoints;
}


void opti_ga::printFitness(vector<genome> &gens, bool first){
  for(auto j : gens){
    cout << j.fitness << " ";
    if(first)
      break;
  }
  cout << endl;
}


// Return elements of vec between start and stop index
template<typename T> vector<T> opti_ga::slice(vector<T> vec, int start_idx, int stop_idx){

  return vector<T>(vec.begin() + start_idx, vec.begin() + stop_idx);
}

//Slice vec and delete sliced elements
template<typename T>
vector<T> opti_ga::sliceErase(vector<T>& vec, int start_idx, int stop_idx){
  auto result = slice<T>(vec, start_idx, stop_idx);
  vec.erase(vec.begin() + start_idx, vec.begin() + stop_idx);
  return result;
}

// Append content of vecB to vecA
template<typename T>
void opti_ga::joinSlices(vector<T>& vecA, vector<T>& vecB){
  int offset = 1;
  vecA.reserve(vecA.size() + vecB.size());
  if(vecA.size() > 2){
    offset = std::experimental::randint(1, (int) vecA.size()-2);
  }
  vecA.insert(vecA.begin() + offset, vecB.begin(), vecB.end());
}

void opti_ga::printWaypoints(vector<Point>& waypoints){
  cout << "Waypoints: " << endl;
  for(const auto i : waypoints){
    cout << i << endl;
  }
}

bool opti_ga::compareFitness(const struct genome &genA, const struct genome &genB){
  return genA.fitness > genB.fitness;
}


void opti_ga::markOcc( Mat &img, Point &start, Point &end, int size, int val=255)
{

  int lineType = LINE_4;
  line( img,
    start,
    end,
    Scalar( val ),
    size -2,
    lineType );
}




void opti_ga::markPath(struct genome &gen)
{

  auto iter = gen.waypoints.begin();
  Point current = *iter;
  iter++;
  do{
    opti_ga::markOcc(*gen.map, current, *iter, 1, 0);
    current = *iter;
    // cout << current << endl;
    iter++;
  } while(iter != gen.waypoints.end());
}




// Methods:
//--------------------------------------------------
void opti_ga::GenPool::populatePool(int size, int waypoints)
{
  // Create initial population of given Size
  auto tbegin = chrono::steady_clock::now();

  for (int i=0; i<size; ++i) {
    struct genome gen;
    gen.map = make_shared<Mat>(Mat(height, width, CV_8U, Scalar(0)));
    gen.waypoints = genWaypoints(width, height, start, waypoints);
    gen.waypoints.push_back(end);

    gen.fitness = this->calFittness(gen);
    this->gens.push_back(gen);
  }
  // auto tend = chrono::steady_clock::now();
  // timer.stop();

  // cout << std::chrono::duration_cast<std::chrono::microseconds>(tend - tbegin).count() << " µs" << endl;
  // cout << timer.elapsed().wall << endl;
  printFitness(gens);
}


double opti_ga::GenPool::calFittness(struct genome &gen)
{
  // Maximize occ minimize time

  // t_start();
  double occ = calOcc(gen);

  // t_end("eval_occ");
  double time = calTime(gen); //- estimation;
  auto robot_cov = robot_size * robot_speed/3.6;
  auto optimal_time = occ/robot_cov;

  double time_err = optimal_time / time;

  double occ_err = occ/(width * height);
  // cout << "Time: " << time_err << " Occ: " << occ_err << endl;

  // t_end("eval_time");
  // double fitness = occ / (abs(time) + 1) - occ_err;
  // double fitness = (occ / sqrt(8*CV_PI)) * exp(-(pow(1-1/time, 2)/8));
  double fitness = fitnessWeight * occ_err + (1-fitnessWeight) * time_err;



  // cout << "Relation" << calOcc(gen) / calTime(gen)  << endl;
  return fitness;
}


double opti_ga::GenPool::calOcc(struct genome &gen)
{

  *gen.map = Scalar(0);
  polylines(*gen.map, gen.waypoints, false, 1, robot_size);
  return ((double) cv::sum(*gen.map)[0]);
}


double opti_ga::GenPool::calTime(struct genome &gen, int speed)
{

  float dist = 0;
  auto iter = gen.waypoints.begin();
  Point current = *iter;
  iter++;
  do{
    // MyLine(map, current, *iter);
    // Sum up all distances the roboter needs to travel
    dist += cv::norm(current - *iter);
    current = *iter;
    // cout << current << endl;
    iter++;
  } while(iter != gen.waypoints.end());

  return dist/ (robot_speed / 3.6);

}


struct genome opti_ga::GenPool::getBest(){
  sort(gens.begin(), gens.end(), compareFitness);
  return gens[0];
}


void opti_ga::GenPool::crossover()
{
  // get best two individuals
  // Assume that gens are already sorted according to their finess
  vector<Point> parent1 = gens[0].waypoints;
  vector<Point> parent2 = gens[1].waypoints;


  int start_node1 = std::experimental::randint(1, (int) parent1.size() - 12);
  int end_node1 = start_node1 + 10;

  // check if lenght is valid
  if(!((start_node1 < (parent2.size()-1))
       && (end_node1 < (parent2.size() -1)))){
    // cout << "--------------------Cancle!" << endl;
    return;
  }

  // int start_node2 = std::experimental::randint(1, (int) parent2.size() - 1);
  // int end_node2 = std::experimental::randint(start_node2, (int) parent2.size() - 1);

  // cout << parent1.size() << " " << start_node1 << " " << end_node1 << endl;


  // cout << "Parent1 before" << endl;
  // printWaypoints(parent1);
  // cout << "--------------------" << endl;
  auto slice1 = sliceErase(parent1, start_node1, end_node1 );
  auto slice2 = sliceErase(parent2, start_node1, end_node1 );
  // auto slice2 = sliceErase(parent2, (int) parent2.size()/2, parent2.size());
  // cout << "Parent1" << endl;
  // printWaypoints(parent1);
  // cout << "--------------------" << endl;
  // cout << "Slice2" << endl;
  // printWaypoints(slice2);


  joinSlices(parent1, slice2);
  joinSlices(parent2, slice1);
  // cout << "--------------------" << endl;
  // cout << "Combined" << endl;
  // printWaypoints(parent1);

  gens[gens.size()-2].waypoints.assign(parent1.begin(), parent1.end());
  gens[gens.size()-1].waypoints.assign(parent2.begin(), parent2.end());

}

void opti_ga::GenPool::conditionalPointShift(Point &p, int magnitude){

  while(true){
    int shift_x = std::experimental::randint(-magnitude, magnitude);
    int shift_y = std::experimental::randint(-magnitude, magnitude);
    if(((p.x + shift_x) > 0)
       && ((p.x + shift_x) < width)
       && ((p.y + shift_y) > 0)
       && ((p.y + shift_y) < height)){
      p.x +=  shift_x;
      p.y +=  shift_y;
      break;
    }
  }
}

void opti_ga::GenPool::randomInsert(struct genome &gen){
  int node = std::experimental::randint(1, (int) gen.waypoints.size()-1);
  Point p(gen.waypoints.at(node));

  conditionalPointShift(p, 10);
  // cout << "Try shift" << endl;
  auto beg = gen.waypoints.begin();
  // cout << "Size before: " << gen.waypoints.size() << endl;
  gen.waypoints.insert(beg + node, p);
  // cout << "Size after: " << gen.waypoints.size() << endl;
  // cout << "Inserted!" << endl;
}

void opti_ga::GenPool::randomRemove(struct genome &gen){
  int node = std::experimental::randint(1, (int) gen.waypoints.size()-2);

  auto beg = gen.waypoints.begin();
  gen.waypoints.erase(beg + node);
  // cout << "Inserted!" << endl;
}

void opti_ga::GenPool::randomSwitch(struct genome &gen){
  int node1 = std::experimental::randint(1, (int) gen.waypoints.size()-2);

  auto tmp =  gen.waypoints[node1];
  int node2 = std::experimental::randint(1, (int) gen.waypoints.size()-2);
  gen.waypoints[node1] = gen.waypoints[node2];
  gen.waypoints[node2] = tmp;
}

void opti_ga::GenPool::mutation(){
  for(int i=1; i<gens.size();i++){
    // randomInsert(gens[i]);
    if (i < 4) {
      randomInsert(gens[i]);

    }else if(i < 6){
      if(gens[i].waypoints.size() > 100)
	randomRemove(gens[i]);

    } else {
      int node = std::experimental::randint(1, (int) gens[i].waypoints.size()-2);
      // if(gens[i].waypoints.size() > 10)
      // 	randomRemove(gens[i]);
      conditionalPointShift(gens[i].waypoints[node], shift_mag);
      randomSwitch(gens[i]);

      // cout << "Test" << endl;
    }
    // gens[i].fitness = calFittness(gens[i]);
  }
}



void opti_ga::GenPool::selection(){


  std::future<double> t_pool[gens.size()];


  for (int i=0; i<gens.size(); ++i) {
    t_pool[i] = std::async([this, i]{return calFittness(gens.at(i));});
  }


  for (int i=0; i<gens.size(); ++i) {
    gens.at(i).fitness = t_pool[i].get();
  }


  // for (int i=0; i<gens.size(); ++i) {
  //   gens.at(i).fitness = calFittness(gens.at(i));
  // }
  // printFitness(gens);
  sort(gens.begin(), gens.end(), compareFitness);
  // printFitness(gens);
}



float opti_ga::GenPool::update(int iterations){
  for (int i = 0; i <= iterations; ++i) {
    // cout << "Crossover" << endl;
    // auto start = timer.start();
    t_start("Crossover");
    crossover();
    t_end();
    // cout << "Mutation" << endl;
    t_start("Mutation");
    mutation();
    t_end();
    t_start("Selection");
    // t_end("mut");
    selection();
    t_end();



    if(i % 1 == 0){
      cout << "Round: " << i << endl;
      // opti_ga::markPath(gens.at(0));
      polylines(*gens.at(0).map, gens.at(0).waypoints, false, 255, robot_size);
      polylines(*gens.at(0).map, gens.at(0).waypoints, false, 1, 1);
      // cv::putText(*gens.at(0).map,"it=" + std::to_string(i) + "Nodes="+std::to_string(gens.at(0).waypoints.size()), Point(10,900), CV_FONT_HERSHEY_SIMPLEX, 1, 255);
      cv::imwrite("res/it_" + std::to_string(i) + "WP_" + to_string(gens.at(0).waypoints.size()) + ".jpg", *gens.at(0).map);
      printFitness(gens, true);
      printTiming();
    }


  }

  printTiming();


  return 42.0;
}
