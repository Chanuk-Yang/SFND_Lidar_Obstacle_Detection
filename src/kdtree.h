/* \author Aaron Brown */
// Quiz on implementing kd tree
#include <cmath>
#include "render/render.h"


// Structure to represent node of kd tree

template<typename PointT>
struct Node
{
	PointT point;
	int id;
	Node<PointT>* left;
	Node<PointT>* right;

	Node(PointT arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

template<typename PointT>
struct KdTree
{
	Node<PointT>* root;

	KdTree()
	: root(NULL)
	{}

	
	void insert(PointT point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		int depth = 0;
		insert_at_node(root, point, id, depth);
	}
  
	void insert_at_node(Node<PointT> *&node, PointT point, int id, int depth)
	{
		if(node == NULL){
			node = new Node<PointT>(point, id);
		}else{
			int cd = depth % 3;
			float crit_point, crit_node_point;
			if (cd==0){
				crit_point = point.x;
				crit_node_point = node->point.x;
			}else if (cd==1){
				crit_point = point.y;
				crit_node_point = node->point.y;
			}else{
				crit_point = point.z;
				crit_node_point = node->point.z;
			}


			if(crit_point < crit_node_point){
				insert_at_node(node->left, point, id, depth+1);
			}else{
				insert_at_node(node->right, point, id, depth+1);
			}
		}
		
	};


	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(PointT target, float distanceTol)
	{
		std::vector<int> ids;
		int depth = 0;
		search_at_node(root, &ids, target, distanceTol, depth);
		return ids;
	}

	void search_at_node(Node<PointT>* node, std::vector<int>* ids, PointT target, float distanceTol, int depth){
		if (node!=NULL){
			float x = node->point.x - target.x;
			float y = node->point.y - target.y;
			float z = node->point.z - target.z;
			int i = depth%3;
			float crit;
			
			if (i==0){
				crit = x;
			}else if (i==1){
				crit = y;
			}else{
				crit = z;
			}

			if (x<distanceTol && x>-distanceTol && y<distanceTol && y>-distanceTol && z<distanceTol && z>-distanceTol){
				float dist = sqrt(x*x + y*y + z*z);
				if (dist <distanceTol)
					ids->push_back(node->id);
			}
			if(crit<distanceTol){
				search_at_node(node->right, ids, target, distanceTol, depth+1);
			}
			if(crit>=-distanceTol){
				search_at_node(node->left, ids, target, distanceTol, depth+1);
			};
			
			
		
		}
		
		
	}

};




