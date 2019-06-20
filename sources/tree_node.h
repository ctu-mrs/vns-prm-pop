/*
 * Node.h
 *
 *  Created on: 1. 2. 2015
 *      Author: Robert Pěnička
 */

#ifndef SRC_PLANNER_TREENODE_H_
#define SRC_PLANNER_TREENODE_H_
#include <vector>
#include <string>
#include <fstream>
#include <thread>
#include "crl/logging.h"
#include "my_defines.h"
//#include "PlannerSpaceSpec.h"
//using imr::logger;
//#define TREE_DATA_TYPE Position3D

template<class T>
class TreeNode {
public:
	TreeNode(bool addTolist = true);
	TreeNode(TreeNode* parent, bool addTolist = true);
	TreeNode(T data, bool addTolist = true);
	TreeNode(T * data, bool addTolist = true);
	virtual ~TreeNode();
	void saveToFile(std::string filename);
	void saveToMatFile(std::string filename, std::string varname);
	void loadFromFile(std::string filename);
	void setData(T data);
	T & getData();
	TreeNode<T> * getParent();
	void setParent(TreeNode<T> * parentNode);
	//std::vector<TreeNode<T> *> getChilder();
	void addChild(TreeNode<T> * child);
	unsigned int getChildCount();
	unsigned int getTotalSubEdgesCount();
	unsigned int getMaxDepth();
	TreeNode<T> * getChild(unsigned int id);
	TreeNode<T> * removeChild(unsigned int id);
	int getId();
	static TreeNode<T> * getTreeNodeById(unsigned int id);
	static int getNodeCount();
	static TreeNode<T> readFromFile(std::string filename);
	TreeNode<T> & operator=(TreeNode<T> & x);
	void copyNode(TreeNode<T> & x);
	void printTree();
private:
	void clearChilder(); //remove all children
	void saveNodeToFile(TreeNode<T> * node, std::ofstream& myfile);
	T data;
	std::vector<TreeNode<T> *> children;
	TreeNode<T> * parent;
	int id;
	static int nodeCount;
	static std::vector<TreeNode<T> *> nodes;
	static pthread_mutex_t * createNewNodeMutex;
	pthread_mutex_t * childrenModMutex;
	bool addedToList;
};

template<class T>
int TreeNode<T>::nodeCount = 0;
template<class T>
std::vector<TreeNode<T> *> TreeNode<T>::nodes;
template<class T>
pthread_mutex_t * TreeNode<T>::createNewNodeMutex = NULL;

template<class T>
TreeNode<T>::TreeNode(bool addTolist) {
	childrenModMutex = new pthread_mutex_t;
	if (pthread_mutex_init(childrenModMutex, NULL) != 0) {
		std::cerr << "pthread_mutex_init childerModMutex failed" << std::endl;
		exit(1);
	}

	this->parent = NULL;
	this->addedToList = addTolist;
	if (this->addedToList) {

		if (TreeNode<T>::createNewNodeMutex == NULL) {
			TreeNode<T>::createNewNodeMutex = new pthread_mutex_t;
			if (pthread_mutex_init(TreeNode<T>::createNewNodeMutex, NULL) != 0) {
				std::cerr << "pthread_mutex_init createNewNodeMutex failed" << std::endl;
				exit(1);
			}
		}

		pthread_mutex_lock(TreeNode<T>::createNewNodeMutex);

		id = TreeNode<T>::nodeCount;
		//INFO("add TreeNode "<<id);
		if (nodes.size() <= nodeCount) {
			int size = nodes.size();
			if (size < 10) {
				size = 10;
			}
			nodes.resize(size * 2);
		}
		nodes[id] = this;
		TreeNode<T>::nodeCount++;

		pthread_mutex_unlock(TreeNode<T>::createNewNodeMutex);
	} else {
		id = -1;
	}

	//INFO("create TreeNode " << id);
}

template<class T>
TreeNode<T>::TreeNode(TreeNode<T> * parent, bool addTolist) :
		TreeNode(addTolist) {
	this->parent = parent;
}

template<class T>
TreeNode<T>::TreeNode(T data, bool addTolist) :
		TreeNode(addTolist) {
	this->data = data;
}

template<class T>
TreeNode<T>::TreeNode(T * data, bool addTolist) :
		TreeNode(addTolist) {
	this->data = *data;
}

template<class T>
TreeNode<T>::~TreeNode() {
	//std::cout << "delete tree node " << this->id << std::endl << std::flush;
	if (this->addedToList) {
		TreeNode::nodeCount--;
	}
	//std::cout << "clear children " << this->id << std::endl << std::flush;
	clearChilder();
}

template<class T>
void TreeNode<T>::setData(T data) {
	this->data = data;
}

template<class T>
T & TreeNode<T>::getData() {
	return this->data;
}

template<class T>
TreeNode<T> * TreeNode<T>::getParent() {
	return this->parent;
}

template<class T>
void TreeNode<T>::setParent(TreeNode* parentNode) {
	this->parent = parentNode;
}

/*
 template<class T>
 std::vector<TreeNode<T> *> TreeNode<T>::getChilder() {
 return this->children;
 }
 */

template<class T>
void TreeNode<T>::addChild(TreeNode* child) {
	pthread_mutex_lock(childrenModMutex);
	this->children.push_back(child);
	pthread_mutex_unlock(childrenModMutex);
}

template<class T>
unsigned int TreeNode<T>::getChildCount() {
	pthread_mutex_lock(childrenModMutex);
	unsigned int size = this->children.size();
	pthread_mutex_unlock(childrenModMutex);
	return size;
}

template<class T>
unsigned int TreeNode<T>::getTotalSubEdgesCount() {
	pthread_mutex_lock(childrenModMutex);
	unsigned int sizeTotal = 0;
	unsigned int sizeThis = this->children.size();
	for (int var = 0; var < sizeThis; ++var) {
		sizeTotal += this->children[var]->getTotalSubEdgesCount();
	}
	sizeTotal += sizeThis;
	pthread_mutex_unlock(childrenModMutex);
	return sizeTotal;
}

template<class T>
unsigned int TreeNode<T>::getMaxDepth() {
	if (this->children.size() > 0) {
		unsigned int maxDepth = 0;
		for (int var = 0; var < this->children.size(); ++var) {
			unsigned int thisDepth = this->children[var]->getMaxDepth();
			if (thisDepth > maxDepth) {
				maxDepth = thisDepth;
			}
		}
		return maxDepth + 1;
	} else {
		return 0;
	}
}

template<class T>
TreeNode<T> * TreeNode<T>::getChild(unsigned int id) {
	pthread_mutex_lock(childrenModMutex);
	TreeNode<T> * child = this->children[id];
	pthread_mutex_unlock(childrenModMutex);
	return child;
}

template<class T>
TreeNode<T> * TreeNode<T>::removeChild(unsigned int id) {
	pthread_mutex_lock(childrenModMutex);
	TreeNode<T> * child = this->children[id];
	this->children.erase(this->children.begin() + id);
	pthread_mutex_unlock(childrenModMutex);
	return child;
}

template<class T>
int TreeNode<T>::getId() {
	return this->id;
}

template<class T>
int TreeNode<T>::getNodeCount() {
	return TreeNode<T>::nodeCount;
}

template<class T>
TreeNode<T> * TreeNode<T>::getTreeNodeById(unsigned int id) {
	return TreeNode<T>::nodes[id];
}

template<class T>
void TreeNode<T>::saveToFile(std::string filename) {
	//INFO("saveToFile " << filename);
	std::ofstream myfile;
	myfile.open(filename.c_str());
	if (myfile.is_open()) {
		myfile << "data=[";
		TreeNode<T>* parent;
		TreeNode<T>* node;
		int nodeID;
		for (int var = 0; var < TreeNode<T>::nodeCount; ++var) {
			node = TreeNode<T>::nodes[var];
			nodeID = node->getId();
			parent = node->getParent();

			myfile << " " << nodeID << " , ";
			if (parent != NULL) {
				int parentID = parent->getId();
				myfile << parentID << " , ";
			} else {
				myfile << nodeID << " , ";
			}
			myfile << node->data << " ;...";
			myfile << std::endl;
		}

		//saveNodeToFile(this, myfile);
		myfile << "];" << std::endl;
		myfile.close();
	}
}

template<class T>
void TreeNode<T>::saveNodeToFile(TreeNode* node, std::ofstream& myfile) {
//INFO("saveNodeToFile " << node);
	int nodeID = node->getId();
	myfile << " " << nodeID << " , ";

	TreeNode<T>* parent = node->getParent();
	if (parent != NULL) {
		int parentID = parent->getId();
		myfile << parentID << " , ";
	} else {
		myfile << nodeID << " , ";
	}

	myfile << node->data << " ;...";
	myfile << std::endl;
	for (int var = 0; var < node->getChildCount(); ++var) {
		TreeNode* child = node->getChild(var);
		//INFO("child " << child->getId());
		saveNodeToFile(child, myfile);
	}
}

template<class T>
void TreeNode<T>::loadFromFile(std::string filename) {

}

template<class T>
TreeNode<T> TreeNode<T>::readFromFile(std::string filename) {

}

template<class T>
void TreeNode<T>::clearChilder() {
	if (children.size() != 0) {
		for (int var = 0; var < children.size(); ++var) {
			delete children[var];
		}
		children.clear();
	}
}

template<class T>
TreeNode<T> & TreeNode<T>::operator=(TreeNode& x) {
	this->clearChilder();
	this->parent = x.getParent();
	for (int var = 0; var < x.getChildCount(); ++var) {
		addChild(x.getChild(var));
	}
	return *this;
}

template<class T>
void TreeNode<T>::copyNode(TreeNode& x) {
	this->clearChilder();
	this->parent = x.getParent();
	this->data = x.getData();
	for (int var = 0; var < x.getChildCount(); ++var) {
		TreeNode* newChild = new TreeNode();
		TreeNode* oldChild = x.getChild(var);
		newChild->setData(oldChild->getData()); //set data
		newChild->copyNode(*oldChild); //copy node
		newChild->setParent(this); //set parent to new tree otherwise it would be old
		newChild->addChild(newChild);
	}
}

template<class T>
void TreeNode<T>::printTree() {
	std::cout << this->data << std::endl;
	;
	for (int var = 0; var < children.size(); ++var) {
		children[var]->printTree();
	}
}

template<class T>
std::ostream & operator <<(std::ostream & o, TreeNode<T> * tn) {
	o << "TreeNode" << tn->getId() << " ";
	return o;
}

#endif /* SRC_PLANNER_TREENODE_H_ */
