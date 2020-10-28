#include "pool.hpp"

vector<Point> getIndex(Point p0, Point p1)
{
  // https://de.wikipedia.org/wiki/Bresenham-Algorithmus
  vector<Point> pts;
  int dx =  abs(p1.x-p0.x), sx = p0.x<p1.x ? 1 : -1;
  int dy = -abs(p1.y-p0.y), sy = p0.y<p1.y ? 1 : -1;
  int err = dx+dy, e2; /* error value e_xy */

  while (1) {
    // map.at<uchar>(p0.x, p0.y) = 255;
    pts.push_back(Point(p0));
    // cout  << p0 << endl;
    if (p0.x==p1.x && p0.y==p1.y) break;
    e2 = 2*err;
    if (e2 > dy) { err += dy; p0.x += sx; } /* e_xy+e_x > 0 */
    if (e2 < dx) { err += dx; p0.y += sy; } /* e_xy+e_y < 0 */
  }
  return pts;
}

vector<Point> getIndex(Size map_size,Point2f p0, Point2f p1,  int size)
{
  Point2f V = p1-p0;
  vector<Point> area;
  // cout << "Start " << V << endl;
  float v_len = cv::norm(V);
  auto Vu = V/v_len;
  // cout << "Start " << Vu << endl;
  Point2f U(-V.y/v_len,V.x/v_len);
  auto start = p0 + U*(size/2);

  for(int i=0; i<v_len; i++){
    // cout << "Idx2: " << start+Vu*i;
    for(auto j=0; j<=size; j++){
      // cout << " | " << (start+Vu*i) - U*j;
      auto tmp = (start+Vu*i) - U*j;
      if(tmp.x >= map_size.width) tmp.x = map_size.width-1;
      if(tmp.x < 0) tmp.x = 0;
      if(tmp.y >= map_size.height) tmp.y = map_size.height-1;
      if(tmp.y < 0) tmp.y = 0;
      area.push_back(Point(round(tmp.x), round(tmp.y)));
    }
    // cout << endl;
  }
  return area;
}




float calOccRec(Mat &map, Point p0, Point p1, int size){
  Point2f V = p1-p0;
  Point V_int = p1-p0;
  // cout << "Start " << V << endl;
  float v_len = cv::norm(V);
  // Calculate unit vector:
  Point2f U(-V.y/v_len,V.x/v_len);
  Point2f Y_unit(0,1) ;

  auto center = (p0 + p1) / 2;
  // Determain the center of the rectangle
  auto dot = V.x*Y_unit.x + V.y*Y_unit.y;

  auto det = V.x*Y_unit.y - V.y*Y_unit.x;

  auto angle = atan2(det, dot)* 180 / CV_PI;
  // cout << "Angle " << angle << endl;

  // for(auto i : pts){
  //   cout << i << endl;
  // }

  // cout << "Center " << center << endl;
  // auto rec = cv::RotatedRect(
  // 			     center,
  // 			     Size2f(size, v_len),
  // 			     -angle
  // 			     );
  // auto rec = cv::RotatedRect(

  auto scale_x = U.x*(size/2);
  auto scale_y = U.y*(size/2);

  Point A0(p1.x - scale_x, p1.y - scale_y);
  Point A1(p1.x + scale_x, p1.y + scale_y);
  Point B0(p0.x - scale_x, p0.y - scale_y);
  Point B1(p0.x + scale_x, p0.y + scale_y);
  auto starts = getIndex(A0, A1);
  // cout << scale_x << " " << scale_y << endl;
  // cout << starts.size() << endl;
  for(auto i : starts){
    getIndex(i, i+V_int);
  }
  return 42;
}

cv::RotatedRect constructRec(Point &p0, Point &p1, int size){
  Point2f V = p1-p0;
  // cout << "Start " << V << endl;
  float v_len = cv::norm(V);
  // Calculate unit vector:
  Point2f U(-V.y/v_len,V.x/v_len);
  Point2f Y_unit(0,1) ;

  auto center = (p0 + p1) / 2;
  // Determain the center of the rectangle
  auto dot = V.x*Y_unit.x + V.y*Y_unit.y;

  auto det = V.x*Y_unit.y - V.y*Y_unit.x;

  auto angle = atan2(det, dot)* 180 / CV_PI;
  // cout << "Angle " << angle << endl;

  // for(auto i : pts){
  //   cout << i << endl;
  // }

  // cout << "Center " << center << endl;
  return cv::RotatedRect(
			 center,
			 Size2f(size, v_len),
			 -angle
			 );

}




void calOccupiedArea(vector<Point> waypoints, int size){

  vector<RotatedRect> recs;
  vector<vector<Point2f>> regions;
  double completeArea = 0;
  for(auto i=0; i<waypoints.size()-1; i++)
    {
      auto currentRec = constructRec(waypoints[i], waypoints[i+1], size);
      for(auto rec : recs){
	vector<Point2f> region;
	// if(norm(currentRec.center-rec.center) < ((currentRec.size.width + rec.size.height)/2)){
	// cout << "Size " << currentRec.size.width << endl;
	if((norm(currentRec.center-rec.center) < ((currentRec.size.height + rec.size.height)/2))
	   ){
	  auto state = cv::rotatedRectangleIntersection(rec, currentRec, region);
	  if(state && region.size() >= 3)
	    {
	      regions.push_back(region);
	    }
	  else{
	    // cout << "Region size: " << region.size() << endl;
	    cout << "Angle: " << currentRec.angle << " | " << rec.angle
		 << " \t-- Angle diff: " << (currentRec.angle - rec.angle)
		 << " \t-- Height " << currentRec.size.height << " | " << rec.size.height
		 << " \t-- Center " << currentRec.center << " | " << rec.center
		 << " \t-- Norm: " << norm(currentRec.center-rec.center)
		 << " \t-- Dist: " << (currentRec.size.height + rec.size.height)/2
		 << endl;
	  }
	}
      }
      recs.push_back(currentRec);
    }

}



// void line(int x0, int y0, int x1, int y1)
// void line(uint8_t map[500][500],  Point p0, Point p1)
void mainline(Mat &map,  Point p0, Point p1, int size)
{
  // https://de.wikipedia.org/wiki/Bresenham-Algorithmus

  int dx =  abs(p1.x-p0.x), sx = p0.x<p1.x ? 1 : -1;
  int dy = -abs(p1.y-p0.y), sy = p0.y<p1.y ? 1 : -1;
  int err = dx+dy, e2; /* error value e_xy */

  while (1) {
    map.at<uchar>(p0.x, p0.y) = 255;
    if (p0.x==p1.x && p0.y==p1.y) break;
    e2 = 2*err;
    if (e2 > dy) { err += dy; p0.x += sx; } /* e_xy+e_x > 0 */
    if (e2 < dx) { err += dx; p0.y += sy; } /* e_xy+e_y < 0 */
  }
}

// void line(Mat map,  Point p0, Point p1, int size){
//   for(auto i=int(-size/2); i < int(size/2); i++){
//     line(map, Point(p0.x+i, p0.y+i), Point(p1.x+i, p1.y+i));
//   }
// }
