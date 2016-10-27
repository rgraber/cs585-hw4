/**
  CS585_Lab3.cpp
  @author: Ajjen Joshi
  @version: 1.0 9/17/2014

  CS585 Image and Video Computing Fall 2014
  Lab 3
  --------------
  This program introduces the following concepts:
  a) Finding objects in a binary image
  b) Filtering objects based on size
  c) Obtaining information about the objects described by their contours
  --------------
  */

#include "libContour.hpp"
using namespace cv;
using namespace std;

// Function header
class State
{
	public:
		Point2d centroid;
		int object_id;
		double area;
		Mat pixels;
};
vector<tuple<State,State> > get_state_matches(vector<State> s1_list,vector<State> s2_list, double (*distance_fn)(State&,State&),double distance_thresh)
{
	vector<tuple<double,int,int> > distances;
	for (int i=0;i<s1_list.size();i++)
	{
		State& s1 = s1_list[i];
		for (int j=0;j<s2_list.size();j++)
		{
			State& s2 = s2_list[j];
			distances.push_back(make_tuple(distance_fn(s1,s2),i,j));
		}
	}
	vector<int> s1_matched = vector<int>(s1_list.size(),-1);
	vector<int> s2_matched = vector<int>(s2_list.size(),-1);
	sort(distances.begin(),distances.end());
	vector<tuple<State,State> > returnme;
	for (int i=0;i<distances.size();i++)
	{
		tuple<double,int,int> t = distances[i];
		double distance = get<0>(t);
		int idx_1 = get<1>(t);
		int idx_2 = get<2>(t);
		cout << distance << ',' << idx_1 << ',' << idx_2 << '\n';
		if ((s1_matched[idx_1] < 0 || s2_matched[idx_2] < 0 )&& distance < distance_thresh)
		//if ( 1 || distance < 1.0)
		{
			s1_matched[idx_1]=1;
			s2_matched[idx_2]=1;
			returnme.push_back(make_tuple(s1_list[idx_1],s2_list[idx_2]));
		}
	}
	return returnme;
}


vector<State> populate_states(Mat& frame)
{
	double mymin,mymax;
	vector<State> states;
	minMaxLoc(frame,&mymin,&mymax,0,0,noArray());
	for (int i=1;i<int(mymax);i++)
	{
		State s;
		Mat tmp = frame==i;
		if (countNonZero(tmp)==0)
		{
			cout << "Gap!\n";
			continue;
		}
		Moments m = moments(tmp,true);
		cout << 'a'<< sum(tmp)[0] << '\n';
		if (countNonZero(tmp) < (15))
		{
			continue;
		}
		Point2d centroid = Point2f(m.m10/m.m00, m.m01/m.m00);
		s.centroid=centroid;
		s.object_id = i;
		s.pixels = frame==i;
		s.area = countNonZero(s.pixels);
		cout << "s:" << i <<',' <<centroid<<'\n';
		states.push_back(s);
	}
	return states;

}

double df2(State& s1, State& s2)
{

	Point2d difference = (s1.centroid - s2.centroid);
	double distance = sqrt(difference.x*difference.x + difference.y*difference.y);
	return distance;
	if (distance > 50)
	{
		return 1.0;
	}
	int intersect_area = countNonZero((s1.pixels > 0 ) & (s2.pixels > 0));
	double s1_area_ratio; 
	double s2_area_ratio;
	if (intersect_area >0){	
		s1_area_ratio=(s1.area / intersect_area);
		s2_area_ratio=(s2.area / intersect_area);
	}
	else
	{
		s1_area_ratio=0;
		s2_area_ratio=0;
	}
	double ratio=0; // Value between 0 and 1.0
	if (s1_area_ratio > s2_area_ratio)
	{
		ratio=s1_area_ratio;
	}
	else
	{
		ratio=s2_area_ratio;
	}
	cout << "nondefault," << ratio << '\n';
	return (1.0-ratio);



}
double df(State& s1, State& s2)
{

	Point2d difference = (s1.centroid - s2.centroid);
	double distance = sqrt(difference.x*difference.x + difference.y*difference.y);
	//return distance;
	if (distance > 50)
	{
		return 1.0;
	}
	int intersect_area = countNonZero((s1.pixels > 0 ) & (s2.pixels > 0));
	double s1_area_ratio; 
	double s2_area_ratio;
	if (intersect_area >0){	
		s1_area_ratio=(s1.area / intersect_area);
		s2_area_ratio=(s2.area / intersect_area);
	}
	else
	{
		s1_area_ratio=0;
		s2_area_ratio=0;
	}
	double ratio=0; // Value between 0 and 1.0
	if (s1_area_ratio > s2_area_ratio)
	{
		ratio=s1_area_ratio;
	}
	else
	{
		ratio=s2_area_ratio;
	}
	cout << "nondefault," << ratio << '\n';
	return (1.0-ratio);



}
bool mergenew(vector <vector <int> >& input, vector <vector <int> >& output , int maxcnt)
{
	vector<int> corr = vector<int>(maxcnt,-1);
	vector<int> cell_corr = vector<int>(input.size(),0);
	vector<vector <int> >cells = vector<vector <int> >(input.size());
	int ctr=0;
	int added=0;
	bool worked=false;
	while(added<input.size()*2)
	{
		if (!worked) // Need to create a new group
		{
			for(int i=0;i<input.size();i++)
			{
				if (cell_corr[i])
				{
					continue;
				}
				cell_corr[i]=1;
				for (int z =0;z<input[i].size();z++)
				{
					corr[input[i][z]] = ctr;
					cells[ctr].push_back(input[i][z]);
					added++;
				}
				ctr++;
				break;

			}
		}
		else
		{
			worked=false;
		}
		for (int i=0; i<input.size();i++)
		{
			if (cell_corr[i])
			{
				continue;
			}
			for (int j=0; j<input[i].size() ; j++)
			{
				int val = input[i][j];
				if (corr[val] != -1)
				{
					for (int z =0;z<input[i].size();z++)
					{
						corr[input[i][z]] = corr[val];
						cells[corr[val]].push_back(input[i][z]);
						added++;
					}
					worked=true;
					cell_corr[i]=1;
					break;
				}
			}
		}

	}


	output=cells;
}


void consolidate_nearby(Mat& in, Mat& out)
{
	vector<State> states = populate_states(in);
	vector<tuple<State,State> > corrs = get_state_matches(states,states,&df2,75.0);
	State a_s, b_s;
	vector<vector<int> > combineme, equivs;
	
	double mymin,mymax;
	minMaxLoc(in,&mymin,&mymax,0,0,noArray());
	vector<State> quicklookup = vector<State>((int)mymax);
	for (int i=0; i<states.size();i++)
	{
		quicklookup[states[i].object_id] = states[i];
	}
	for (int i=0; i<corrs.size(); i++)
	{	
		vector<int> v;
		a_s = get<0>(corrs[i]);
		b_s = get<1>(corrs[i]);
		
		int a = a_s.object_id;
		int b = b_s.object_id;
		v.push_back(a);		
		v.push_back(b);
		combineme.push_back(v);	
	}
	mergenew(combineme,equivs,(int)mymax);
	for (int i=0;i<equivs.size();i++)
	{
		int max=0;
		int max_id=0;
		for (int j = 0;j<equivs[i].size();j++)
		{
			out.setTo((1+i),in==equivs[i][j]);
			int area = quicklookup[equivs[i][j]].area;
			if (area > max)
			{
				max=area;
				max_id = quicklookup[equivs[i][j]].object_id;
			}
		}
		out.setTo(max_id,out==(1+i));
	}

}


void colorize(Mat& in, Mat& out)
{
	double mymin,mymax;
	minMaxLoc(in,&mymin,&mymax,0,0,noArray());
	vector<Mat> colors;
	colors.push_back(Mat::zeros(in.rows,in.cols,CV_8U));
	colors.push_back(Mat::zeros(in.rows,in.cols,CV_8U));
	colors.push_back(Mat::zeros(in.rows,in.cols,CV_8U));
	for(int i=1;i<int(mymax);i++)
	{
		uchar r,g,b;

		b= ((1+i)*37221) %255;
		g= ((1+i)*9157) %255;
		r= ((1+i)*127993) %255;
		colors[0].setTo((b),in==i);
		colors[1].setTo((g),in==i);
		colors[2].setTo((r),in==i);
		//		printf("%d:%d,%d,%d\n",i,b,g,r);
	}
	merge(colors,out);
	//	cout <<"aaa" <<out.type() << '\n';
}


bool eq_label(Vec3i a, Vec3i b)
{
	return (a[1]==b[1]) && a[1] > 0;
}
bool eq_point(Vec3i a, Vec3i b)
{
	return (a[0]==b[0]);
}
bool is_labeled(Vec3i a)
{
	return a[1] != 0;
}
void add_equality(vector<vector <int> >& list,Vec3i a,Vec3i b)
{
	if ((a[1] != 0) && (b[1] !=0))
	{
		vector <int> v;
		v.push_back(a[1]);
		v.push_back(b[1]);
		list.push_back(v);
	}
	else
	{
		cout << "This shouldn't happen! Tried to set a label equality between label and no label\n";
	}
	return;
}
void copy_label(Vec3i& src, Vec3i& dst)
{
	dst[1]=src[1];
	return;
}
void make_sticky(Mat& prev,Mat& curr,Mat& out)
{
	Mat mask = (prev >0) & (curr > 0);
	SparseMat combo = SparseMat(mask);
	vector<State> states_prev = populate_states(prev);		   
	vector<State> states_curr = populate_states(curr);
	vector<tuple<State,State> > corrs = get_state_matches(states_prev,states_curr,&df,25);
	out = Mat::zeros(curr.rows,curr.cols,CV_32S);
	//curr.copyTo(out);
	
	
	double myminc,mymaxc;
	double myminp,mymaxp;
	minMaxLoc(curr,&myminc,&mymaxc,0,0,noArray());
	minMaxLoc(curr,&myminp,&mymaxp,0,0,noArray());
	int realmax;
	if (int(mymaxp) > int(mymaxc))
	{
		realmax = int(mymaxp);
	}
	else
	{
		realmax = int(mymaxc);
	}

	vector<int> seen = vector<int>(realmax,-1); //Maps values in prev frame onto current frame values
	
	
	// Handles direct correspondences
	Mat outd;
	out.copyTo(outd);
	for (int i=corrs.size()-1;i>=0;i--)
	{
		State prev_s,curr_s;
		prev_s = get<0>(corrs[i]);
		curr_s = get<1>(corrs[i]);


		int p = prev_s.object_id;
		int v = curr_s.object_id;
		if (seen[p] != -1 && seen[p] !=v)
		{
			out.setTo(v,curr==v);		
			//line(outd,prev_s.centroid,curr_s.centroid,Scalar(v),2);
			continue;
		}
		seen[p] = v;

		Mat innermask = curr==v;
		out.setTo(p,innermask);
		//line(outd,prev_s.centroid,curr_s.centroid,Scalar(p),2);
		//circle(out,prev_s.centroid,3,Scalar((oid*127)%255,(oid*43)%255,(oid*87)%255),-1);
		//circle(out,curr_s.centroid,3,Scalar((oid*127)%255,(oid*43)%255,(oid*87)%255),-1);
	//	out.setTo(prev_s.object_id,curr==curr_s.object_id);

	}
	SparseMat leftover = SparseMat((curr >0) - (out > 0));
//	Mat _outd;
//	colorize(outd,_outd);
//	imshow("Debug", (curr >0) - (out > 0) );
//	imshow("Debug2", _outd);
	for(auto it = leftover.begin<uchar>(); it != leftover.end<uchar>(); ++it)
	{

		int x=it.node()->idx[0];
		int y=it.node()->idx[1];
		int v = curr.at<int>(x,y);
		out.setTo(v+realmax,curr==v);
	}
	Mat _out = Mat::zeros(out.size(),CV_32S);
	consolidate_nearby(out,_out);	
	out=_out;
	return;
/*
	for(auto it = combo.begin<uchar>(); it != combo.end<uchar>(); ++it)
	{
		int x=it.node()->idx[0];
		int y=it.node()->idx[1];

		int p = prev.at<int>(x,y);
		int v = curr.at<int>(x,y);
		if (seen[p] != -1 && seen[p] !=v)
		{
			out.setTo(v,curr==v);			
			continue;
		}
		seen[p] = v;

		Mat innermask = curr==v;
		out.setTo(p,innermask);
	}
	SparseMat leftover = SparseMat((curr >0) - (out > 0));
	for(auto it = leftover.begin<uchar>(); it != leftover.end<uchar>(); ++it)
	{

		int x=it.node()->idx[0];
		int y=it.node()->idx[1];
		int v = curr.at<int>(x,y);
		out.setTo(v+realmax,curr==v);
	}
*/
}

bool mergecells(vector<unordered_set <int> >& input)
{
	bool breakout=false;
	vector <unordered_set <int> > output;
	vector <int> taken = vector<int>(input.size());
	for (int i=0;i<input.size();i++)
	{
		unordered_set<int> current;
		if (taken[i]==0)
			current = input[i];
		else
			continue;
		for (int j=0;j<input.size();j++)
		{
			if (j==i || taken[j])
				continue;	
			if (any_of(input[j].begin(),input[j].end(),[input,i](int x){return input[i].count(x);}))
			{
				input[i].insert(input[j].begin(),input[j].end());
				taken[j]=1;
				breakout=true;
			}
		}
	}
	for (int i=0;i<taken.size();i++)
	{
		if(taken[i])
			input[i] = unordered_set<int>();
	}

	return breakout;
}
void myFindContours(Mat& input, Mat& _output)
{
	Mat square,tmp;
	input.convertTo(tmp,CV_32S);
	vector <Mat> mergeme;
	mergeme.push_back(tmp);
	mergeme.push_back(Mat::zeros(tmp.rows,tmp.cols,CV_32S));
	mergeme.push_back(Mat::zeros(tmp.rows,tmp.cols,CV_32S));
	merge(mergeme,tmp);
	tmp &= 1;

	vector<Mat> colors;
	vector <vector <int> > equalities;
	int ctr=1;
	for(int i=0; i<tmp.rows -1;i++)
	{
		for(int j=0; j<tmp.cols-1; j++)
		{
			//			cout << i << ',' << j << ','<<tmp.cols <<',' << tmp.rows<<'\n';
			//			square = tmp(Rect(Point2d(j,i),Size(2,2)));
			Vec3i& d= tmp.at<Vec3i>(0+i,0+j);
			Vec3i& c= tmp.at<Vec3i>(0+i,1+j);
			Vec3i& b= tmp.at<Vec3i>(1+i,0+j);
			Vec3i& a= tmp.at<Vec3i>(1+i,1+j);
			//cout << a[0] << ',';
			//cout << b[0] << ',';
			//cout << c[0] << ',';
			//cout << d[0] << ',' << '\n';
			if (a[0] == 0)
			{
				continue;
			}
			if (eq_point(a,d) && is_labeled(d))
			{
				//cout << a[0] << "==" << d[0] << ',' << "Copying 
				copy_label(d,a);
				//Set A's label equal to D's label
				continue;
			}
			if (eq_point(a,b) && is_labeled(b))
			{
				if (eq_point(a,c))	
				{
					if (!eq_label(b,c) && is_labeled(b) && is_labeled(c))
					{

						add_equality(equalities, b,c);
						//UPDATE
					}
				}
				copy_label(b,a);
				// Set A's label equal to B's label
				continue;
			}

			if (eq_point(a,c) && is_labeled(c))
			{

				copy_label(c,a);
				//Set A's label equal to C's label
				continue;
			}
			//cout << ctr << '\n';
			a[1] = ctr;
			add_equality(equalities,a,a);
			ctr++;
			// Set A's label to new label and increment counter
			continue;

		}
	}

	vector <vector <int> > equivs;
	mergenew(equalities,equivs,ctr);


	split(tmp,colors);
	/*	vector <int> current;
		bool breakout=false;
		unordered_map<int, unordered_set<int> > m;
		int a,b;
		for (int i=0;i<equalities.size();i++)
		{
		a = equalities[i][0];
		b = equalities[i][1];
		m[a].emplace(b);
		m[b].emplace(b);
		m[b].emplace(a);
		m[a].emplace(a);
		}
		*/	
	Mat output = Mat::zeros(input.rows,input.cols,CV_32S);
	/*
	   vector<unordered_set <int> > agg;
	   for ( auto it = m.begin(); it != m.end(); it++ )
	   {
	   agg.push_back(it->second);	

	   }
	   vector<unordered_set <int> > mergeit = agg;
	   int prev = 0;
	   while(1)
	   {
	   if(!mergecells(mergeit))
	   break;	
	   }
	   */	
	for (int i=0;i<equivs.size();i++)
	{
		for (int j = 0;j<equivs[i].size();j++)
		{
			output.setTo((1+i),colors[1]==equivs[i][j]);
		}
	}

	_output = output;
}

