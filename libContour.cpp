/*
 * libContour.cpp
 * Authors: Danny Cooper, Rebecca Graber, Megan Van Welie
 * Purpose: Library for identifying and coloring connected components
 */


#include "libContour.hpp"
using namespace cv;
using namespace std;

// Function header

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

for (int i=0;i<cells.size();i++)
{
	if (cells[i].size())
		cout << i << ':';
	for (int j=0;j<cells[i].size();j++)
	{
		cout << cells[i][j] << ',';
	}
	if (cells[i].size())
		cout << '\n';
}
output=cells;
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
	vector <vector <int> > cells = equivs;
	for (int i=0;i<cells.size();i++)
	{
		if (cells[i].size())
			cout << i << ':';
		for (int j=0;j<cells[i].size();j++)
		{
			cout << cells[i][j] << ',';
		}
		if (cells[i].size())
			cout << '\n';
	}

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

