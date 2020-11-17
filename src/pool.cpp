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


bool opti_ga::GenPool::eventOccurred(double probability){
  return probability >= distribution(generator);
}

void printGen(genome gen){
  for(auto w:gen.waypoints){
    cout << w << " | ";
  }
  cout << endl;
  std::cout << "Fitness " << gen.fitness << "\n";

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

    // gen.fitness = this->calFittness(gen);
    this->gens.push_back(gen);
  }

  printFitness(gens);
}


double opti_ga::GenPool::calFittness(struct genome &gen)
{
  // Maximize occ minimize time

  // t_start();
  double occ = calOcc(gen);

  // t_end("eval_occ");
  double time = calTime(gen); //- estimation;
  auto robot_cov = robot_size * robot_speed;
  auto optimal_time = occ/robot_cov;

  if(time < optimal_time)
    throw __LINE__;
    // cout << "Time: " <<  time << endl;
    // throw_line("Time limit failed! Time: " + to_string(time) + " optimal time: " + to_string(optimal_time));

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
  polylines(*gen.map, gen.waypoints, false, 255, robot_size);
  double pix_sum = ((double) cv::sum(*gen.map)[0]) / 255;
  // imshow("Negation", *gen.map);
  // waitKey();
  // bitwise_not(*gen.map, *gen.map);
  // vector<vector<Point> > contours;
  // findContours(*gen.map, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
  // double regionCount = contours.size();

  return pix_sum;
}


int opti_ga::GenPool::detUncoverdRegions(Mat &map){

}


double opti_ga::GenPool::calTime(struct genome &gen, int speed)
{

  double dist = 0;
  double rotPanelty = 0;
  Point prev = gen.waypoints.at(0);
  Point current = gen.waypoints.at(1);
  auto iter = gen.waypoints.begin() + 2;

  // iter++;
  do{
    // MyLine(map, current, *iter);
    // Sum up all distances the roboter needs to travel
    Point A(current - prev);
    Point B(*iter - current);

    double normA = norm(A);
    double normB = norm(B);

    double denum = normA * normB;
    if (denum != 0){
      double innerProd = A.dot(B)/denum;
      // cout << "Check " << (trunc(innerProd) == -1.0) << endl;
      if(trunc(innerProd) == -1.0){
	// cout << "Going back" << endl;
	rotPanelty += 180;
      }
      else if (trunc(innerProd) == 1.0){
	// cout << "Going straight" << endl;
      }
      else {
	rotPanelty += acos(innerProd)*(180/M_PI);
      }
      // printGen(gen);
      if(isnan(rotPanelty)){

	cout << "Rot " << rotPanelty << endl;
	cout << "Inner Product: " << innerProd << endl;
	cout << "A: " << A << " B: " << B  << " prev "<< prev << "Current" << current  << "next" << *iter << endl;
	// cout << "Rot: " << rotPanelty << endl;
	cout << __LINE__ << endl;
	// throw_line("Rot Panelty is NaN.");
	throw __LINE__;
	// throw std::runtime_error(to_string(__FILE__)+ ":" + to_string( __LINE__));
      }
     }
    // Calculate angle between A and B and add it to the rotation

    // cout << "Panelty: " <<  rotPanelty << endl;
    dist += normB;
    prev = current;
    current = *iter;
    // cout << current << endl;
    iter++;
  } while(iter != gen.waypoints.end());

  // cout << "Panelty: " <<  rotPanelty << endl;
  return (dist )/ (robot_speed / 3.6) + rotPanelty;

}



struct genome opti_ga::GenPool::getBest(){
  sort(gens.begin(), gens.end(), compareFitness);
  return gens[0];
}


void opti_ga::GenPool::crossover(genome &gen1, genome &gen2)
{

  vector<Point> parent1(gen1.waypoints.size() > gen2.waypoints.size() ? gen2.waypoints : gen1.waypoints);
  vector<Point> parent2(gen1.waypoints.size() > gen2.waypoints.size() ?  gen1.waypoints : gen2.waypoints);

  int sliceNode = std::experimental::randint(1, (int) parent1.size() - 1);
  vector<Point> child1(parent1.begin(), parent1.begin()+sliceNode), child2(parent2.begin(), parent2.begin()+sliceNode);

  // child1.resize(parent2.size());
  // child2.resize(parent1.size());
  child1.insert(child1.begin() + sliceNode, parent2.begin()+sliceNode, parent2.end());
  child2.insert(child2.begin() + sliceNode, parent1.begin()+sliceNode, parent1.end());


  addGen(child1);
  addGen(child2);

}

void opti_ga::GenPool::addGen(vector<Point> &waypoints) {
  if(unusedMaps.size() > 0){
    // We can recycle an old map
  } else {
    genome gen(make_shared<Mat>(Mat(height, width, CV_8U, Scalar(0))), waypoints);
    gens.push_back(gen);
  }
}

void opti_ga::GenPool::conditionalPointShift(Point &p, int magnitude){

  auto timeout = 0;
  while(true){
    int shift_x = randgen(generator);
    int shift_y = randgen(generator);
    // std::cout << "shift x: " << shift_x << "shift_y" << shift_y  << "\n";
    // std::cout << p << "\n";

    if(timeout > 100){
      throw 42;
    }else{
      timeout++;
    }

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
  for(int i=0; i<gens.size();i++){

    if (eventOccurred(0.01))
      randomInsert(gens[i]);

    if(eventOccurred(0.01) && gens[i].waypoints.size() > 10)
	randomRemove(gens[i]);

    if(eventOccurred(0.5))
      randomSwitch(gens[i]);

    // TODO: for each genome
    for (auto &node : gens[i].waypoints){

      if (eventOccurred(0.45) && node != gens[i].waypoints.front() && node != gens[i].waypoints.back()) {
	// int node = std::experimental::randint(1, (int) gens[i].waypoints.size()-2);
	conditionalPointShift(node, 20);
      }
    }
  }
}


void opti_ga::GenPool::selection(){


  vector<genome> best(gens.begin(), gens.begin() + offspringCount*2);

  for(auto i=0; i<offspringCount; i++){
    // std::cout << "gens: " <<gens.size() << "\n";
    // std::cout << "mating: " << matingPool.size() << "\n";
    matingPool.push_back(roulettWheelSelection());
    // std::cout << "gens: " <<gens.size() << "\n";
  }


  // throw 32;
  // clear the GenPool
  // cout << "Fill population with parents" << endl;
  gens.clear();
  gens.insert(gens.begin(), best.begin(), best.end());
  // std::cout << "Size after clea: " << gens.size() << "\n";


  // build the new population
  // std::cout << "Mating Pool" << "\n";

  // for(auto gen : matingPool){
  //   printGen(gen);
  // }
  // throw 32;

  // cout << "Fill population with children" << endl;
  for(auto j=0; j<matingPool.size()-1; j++){
    auto par = matingPool.at(j);
    // std::cout << par.fitness << "\n";

    // matingPool.pop_back();
    for(auto k=j+1; k<matingPool.size()-1; k++){
      // std::cout << "Before crossover"  << "\n";
      // cout << pool_to_string();
      crossover(par, matingPool.at(k));
      // cout << pool_to_string();
      // std::cout << "After crossover"  << "\n";
      // throw 43;
    }
  }

  // std::cout << "Clear mating pool" << "\n";

  matingPool.clear();

}

/*
Select a gen based on their fitness probability
After a gen is selected it is removed from the pool.
 */
genome opti_ga::GenPool::roulettWheelSelection(){
  // Calculate Probabilities for all individuals
  // std::default_random_engine generator;
  // std::uniform_real_distribution<double> distribution(0.0,1.0);

  double totalFitness = 0;
  for(auto &gen :gens){
    totalFitness += gen.fitness;
  }

  double rand = distribution(generator);
  // cout << "Wheel: " << rand << endl;
  double offset = 0.0;
  genome gen;
  for(int i=0; i < gens.size(); i++){
    offset += gens.at(i).fitness / totalFitness;
    // std::cout << "Fitness: " << gens.at(i).fitness << "\n";
    // std::cout << "total Fitness: " << totalFitness << "\n";
    // std::cout << "Division " << gens.at(i).fitness / totalFitness << "\n";


    // std::cout << "offset: " << offset << "\n";
    // std::cout << "random val: " << rand << "\n";

    if(rand < offset){
      // std::cout << "Gen found!!" << "\n";
      // std::cout << "Size before erase: " << gens.size() << "\n";


      gen = gens.at(i);
      gens.erase(gens.begin() + i);
      // std::cout << "Selected Gen: " << "\n";
      // for(auto w : gen.waypoints){
      // 	cout << w << " | ";
      // }
      // cout << endl;
      // cout<< gen.fitness << endl;

      // std::cout << "Size after erase: " << gens.size() << "\n";
      break;
      // return gen;
    }
  }
  // throw 32;
  return gen;
}


void fixWaypoints(genome &gen){

  // auto current = gen.waypoints.begin() + 1;
  // cout << "--" << endl;
  for(auto i=gen.waypoints.begin(); i != gen.waypoints.end();){
    if(*i == *(i+1)){
      gen.waypoints.erase(i);
      // if(i != gen.waypoints.begin()){
      // 	i--;}
      // cout << "Erase: " << *i << endl;
    }else{
      i++;
    }
  }
  // std::cout << "Before: " << gen.waypoints.size() << "\n";

  // gen.waypoints.erase(std::unique(gen.waypoints.begin(), gen.waypoints.end()), gen.waypoints.end());
}

void opti_ga::GenPool::updateFitness(){


  std::future<double> t_pool[gens.size()];
  // cout << "Population size: " << gens.size() << endl;

  for (int i=0; i<gens.size(); ++i) {
    auto gen = gens.at(i);
    t_pool[i] = std::async([this, i]{
      auto gen = gens.at(i);
      // fixWaypoints(gen);
      return calFittness(gen);
    });
  }


  for (int i=0; i<gens.size(); ++i) {
    gens.at(i).fitness = t_pool[i].get();
  }


  // for (int i=0; i<gens.size(); ++i) {
  //   // fixWaypoints(gens.at(i));
  //   gens.at(i).fitness = calFittness(gens.at(i));

  // }
  // printFitness(gens);
  sort(gens.begin(), gens.end(), compareFitness);
   // printFitness(gens);
}



float opti_ga::GenPool::update(int iterations){
  cout << "start:" << endl;
  // cout << pool_to_string();
  updateFitness();
  for (int i = 0; i <= iterations; ++i) {
    // cout << "Crossover" << endl;
    // auto start = timer.start();

    t_start("Selection");
    // std::cout << "Selection" << "\n";
    selection();
    t_end();
    // std::cout << "Fitness" << "\n";
    // cout << "Mutation" << endl;
    // cout << this->pool_to_string();
    // throw 42;
    t_start("Mutation");
    mutation();
    t_end();
    t_start("updateFitness");
    // std::cout << " Update Fitness" << "\n";

    updateFitness();

    // cout << pool_to_string();
    // cout << pool_to_string();
    t_end();

    if(i % 10 == 0){
      cout << "Round: " << i << "\t";
      // opti_ga::markPath(gens.at(0));
      polylines(*gens.at(0).map, gens.at(0).waypoints, false, 255, robot_size);
      polylines(*gens.at(0).map, gens.at(0).waypoints, false, 0, 1);
      // cv::putText(*gens.at(0).map,"it=" + std::to_string(i) + "Nodes="+std::to_string(gens.at(0).waypoints.size()), Point(10,900), CV_FONT_HERSHEY_SIMPLEX, 1, 255);
      cv::imwrite("res/it_" + std::to_string(i) + "WP_" + to_string(gens.at(0).waypoints.size()) + ".jpg", *gens.at(0).map);
      printFitness(gens, true);
      // printTiming();
    }
  }

  printTiming();


  return 42.0;
}
