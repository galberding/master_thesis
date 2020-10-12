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


void opti_ga::printFitness(vector<genome> &gens){
  for(auto j : gens){
    cout << j.fitness << " ";
    // break;
  }
  cout << endl;
}


// Return elements of vec between start and stop index
template<typename T> vector<T> opti_ga::slice(vector<T>& vec, int start_idx, int stop_idx){

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
  vecA.reserve(vecA.size() + vecB.size());
  vecA.insert(vecA.end(), vecB.begin(), vecB.end());
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


void opti_ga::markOcc( Mat &img, Point &start, Point &end )
{
  int thickness = 4;
  int lineType = LINE_4;
  line( img,
    start,
    end,
    Scalar( 255 ),
    thickness,
    lineType );
}




// Methods:
//--------------------------------------------------
void opti_ga::GenPool::populatePool(int size, int waypoints)
{
  // Create initial population of given Size

  for (int i=0; i<size; ++i) {
    struct genome gen;
    gen.map = make_shared<Mat>(Mat(height, width, CV_8U, Scalar(0)));
    gen.waypoints = genWaypoints(width, height, start, waypoints);
    gen.waypoints.push_back(end);

    gen.fitness = this->calFittness(gen);
    this->gens.push_back(gen);
  }
  printFitness(gens);
}


float opti_ga::GenPool::calFittness(struct genome &gen)
{
  // Maximize occ minimize time
  return calOcc(gen)/100 - calTime(gen)*50;
}



float opti_ga::GenPool::calOcc(struct genome &gen)
{

  *gen.map = Scalar(0);
  // cout << "Initial cost: " << cv::sum(map) << endl;

  auto iter = gen.waypoints.begin();
  Point current = *iter;
  iter++;
  do{
    opti_ga::markOcc(*gen.map, current, *iter);
    current = *iter;
    // cout << current << endl;
    iter++;
  } while(iter != gen.waypoints.end());
  return cv::sum(*gen.map)[0] ; // / (width * height);
}


float opti_ga::GenPool::calTime(struct genome &gen, int speed)
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

  return dist / speed;

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

  auto slice1 = sliceErase(parent1, (int) parent1.size()/2, parent1.size());
  auto slice2 = sliceErase(parent2, (int) parent2.size()/2, parent2.size());

  joinSlices(parent1, slice2);
  joinSlices(parent2, slice1);

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

  conditionalPointShift(p, 200);
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


void opti_ga::GenPool::mutation(){
  for(int i=1; i<gens.size();i++){
    // randomInsert(gens[i]);
    if (i < 3) {
      randomInsert(gens[i]);

    }else if(i == 4){
      if(gens[i].waypoints.size() > 100)
	randomRemove(gens[i]);
    } else {
      int node = std::experimental::randint(1, (int) gens[i].waypoints.size()-2);
      // if(gens[i].waypoints.size() > 10)
      // 	randomRemove(gens[i]);
      conditionalPointShift(gens[i].waypoints[node], 50);
      // cout << "Test" << endl;
    }
    // gens[i].fitness = calFittness(gens[i]);
  }
}

void opti_ga::GenPool::selection(){
  for (int i=0; i<gens.size(); ++i) {
    gens.at(i).fitness = calFittness(gens.at(i));
  }
  printFitness(gens);
  sort(gens.begin(), gens.end(), compareFitness);
  printFitness(gens);
}



float opti_ga::GenPool::update(int iterations){
  for (int i = 0; i <= iterations; ++i) {
    // cout << "Crossover" << endl;
    crossover();
    // cout << "Mutation" << endl;
    mutation();
    // cout << "Selection" << endl;
    selection();
    if(i % 1000 == 0){
      cout << "Round: " << i << endl;
      cv::putText(*gens.at(0).map,"it=" + std::to_string(i) + "Nodes="+std::to_string(gens.at(0).waypoints.size()), Point(10,900), CV_FONT_HERSHEY_SIMPLEX, 1, 255);
      cv::imwrite("res/it_" + std::to_string(i) + "WP_" + to_string(gens.at(0).waypoints.size()) + ".jpg", *gens.at(0).map);
    }
  }
  return 42.0;
}
