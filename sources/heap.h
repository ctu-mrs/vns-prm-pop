/*
 * Heap.h
 *
 *  Created on: 3. 2. 2015
 *      Author: Robert Pěnička
 */

#ifndef SRC_PLANNER_HEAP_H_
#define SRC_PLANNER_HEAP_H_
#include "vector"
#include "crl/logging.h"
#include <cmath>
#include <set>
#include "my_defines.h"
#include "opendubins/dubins.h"

using namespace opendubins;
using crl::logger;

template<typename T>
struct HeapNode;

typedef struct HeapPoint2D {
	double x;
	double y;
	HeapPoint2D() {
		this->x = NAN;
		this->y = NAN;
	}

	HeapPoint2D(double x_, double y_) {
		this->x = x_;
		this->y = y_;
	}

	std::vector<double> toVector() {
		std::vector<double> vector;
		vector.push_back(x);
		vector.push_back(y);
		return vector;
	}
	double distance(HeapPoint2D other) {
		double diffx = this->x - other.x;
		double diffy = this->y - other.y;
		return sqrt(diffx * diffx + diffy * diffy);
	}

	HeapPoint2D getStateInDistance(HeapPoint2D to, double in_distance) {
		double mutual_distance = this->distance(to);
		double x_ = this->x + (to.x - this->x) * (in_distance / mutual_distance);
		double y_ = this->y + (to.y - this->y) * (in_distance / mutual_distance);
		return HeapPoint2D(x_, y_);
	}

	Point3D toPoint3D() {
		return Point3D(this->x, this->y, 0);
	}
} HeapPoint2D;

typedef struct HeapPoint2DHeading {
	double x;
	double y;
	double phi;
	double radius;

	HeapPoint2DHeading() {
		this->x = NAN;
		this->y = NAN;
		this->phi = NAN;
		this->radius = NAN;
	}

	HeapPoint2DHeading(double x_, double y_, double phi_, double radius_) {
		this->x = x_;
		this->y = y_;
		this->phi = phi_;
		this->radius = radius_;
	}

	std::vector<double> toVector() {
		std::vector<double> vector;
		vector.push_back(x);
		vector.push_back(y);
		vector.push_back(phi);
		return vector;
	}
	double distance(HeapPoint2DHeading other) {
		//INFO("dubins distance");
		Dubins dub(State(this->x, this->y, this->phi), State(other.x, other.y, other.phi), this->radius);
		return dub.length;
	}

	HeapPoint2DHeading getStateInDistance(HeapPoint2DHeading to, double in_distance) {
		Dubins dub(State(this->x, this->y, this->phi), State(to.x, to.y, to.phi), this->radius);
		State state_in_distance = dub.getState(in_distance);
		return HeapPoint2DHeading(state_in_distance.point.x, state_in_distance.point.y, state_in_distance.ang,
				this->radius);
	}
	Point3D toPoint3D() {
		return Point3D(this->x, this->y, 0);
	}
} HeapPoint2DHeading;

typedef struct HeapPoint3D {
	double x;
	double y;
	double z;

	HeapPoint3D() {
		this->x = NAN;
		this->y = NAN;
		this->z = NAN;
	}

	HeapPoint3D(double x_, double y_, double z_) {
		this->x = x_;
		this->y = y_;
		this->z = z_;
	}

	std::vector<double> toVector() {
		std::vector<double> vector;
		vector.push_back(x);
		vector.push_back(y);
		vector.push_back(z);
		return vector;
	}
	double distance(HeapPoint3D other) {
		double diffx = this->x - other.x;
		double diffy = this->y - other.y;
		double diffz = this->z - other.z;
		return sqrt(diffx * diffx + diffy * diffy + diffz * diffz);
	}
	HeapPoint3D getStateInDistance(HeapPoint3D to, double in_distance) {
		double mutual_distance = this->distance(to);
		double x_ = this->x + (to.x - this->x) * (in_distance / mutual_distance);
		double y_ = this->y + (to.y - this->y) * (in_distance / mutual_distance);
		double z_ = this->z + (to.z - this->z) * (in_distance / mutual_distance);
		return HeapPoint3D(x_, y_, z_);
	}

	Point3D toPoint3D() {
		return Point3D(this->x, this->y, this->z);
	}
} HeapPoint3D;

template<typename T>
struct HeapNode {
	int node_id;
	int cluster_id;
	T data;
	std::map<HeapNode<T>*, double> visibility_node_ids;
	double distance_from_start;
	HeapNode<T> * previous_point;
	int heap_position; //bool visited;
	bool city_node;

};

//#define HEAPCOST_GET(i) heapVector[i]->getDistanceFromStart()
//#define HEAPCOST_SET(i,cost) heapVector[i]->setDistanceFromStart( cost )
#define LCHILD(x) 2 * x + 1
#define RCHILD(x) 2 * x + 2
#define PARENT(x) (x - 1) / 2

#define HEAPCOST_GET(i) (heapVector[i]->distance_from_start)
#define HEAPCOST_SET(i,cost) {heapVector[i]->distance_from_start = cost;}
#define HEAP_SET_ELEMENT_POSITION(element,i) {element->heap_position = i;}
#define HEAP_GET_ELEMENT_POSITION(element) (element->heap_position)

template<class T>
class Heap {
public:
	Heap();
	Heap(T* array, int arrayLength);
	Heap(std::vector<T> data);
	~Heap();
	void push(T newValue);
	void updateCost(int position, double cost);
	void updateCost(T value, double cost);
	T pop();
	T get();
	T get(int id);
	void replace(int id, T newValue);
	int size();
	void clear();
	bool empty();
	bool checkOrdering();
	bool checkIndex(int index);
	std::vector<T> getHeapVector();
private:
	void sort();
	void BubbleDown(int index);
	void BubbleUp(int index);
	std::vector<T> heapVector;

};

template<class T>
Heap<T>::Heap() {

}

template<class T>
Heap<T>::Heap(T* array, int arrayLength) :
		heapVector(arrayLength) {
	for (int var = 0; var < arrayLength; ++var) {
		heapVector[var] = array[var];
		HEAP_SET_ELEMENT_POSITION(heapVector[var], var);
	}
}

template<class T>
Heap<T>::Heap(std::vector<T> data) :
		heapVector(data) {
	for (int var = 0; var < data.size(); ++var) {
		HEAP_SET_ELEMENT_POSITION(heapVector[var], var);
	}
	sort();
}

template<class T>
Heap<T>::~Heap() {
	// TODO Auto-generated destructor stub
}

template<class T>
void Heap<T>::sort() {
	int size = this->heapVector.size();
	for (int var = size - 1; var >= 0; --var) {
		BubbleDown(var);
	}
}

template<class T>
void Heap<T>::BubbleDown(int index) {
	int size = heapVector.size();

	int leftChildIndex = LCHILD(index);
	int rightChildIndex = RCHILD(index);
	if (leftChildIndex >= size) {
		return; //index is a leaf
	}
	int minIndex = index;
	if ( HEAPCOST_GET(index) > HEAPCOST_GET(leftChildIndex)) {
		minIndex = leftChildIndex;
	}

	if ((rightChildIndex < size) && (HEAPCOST_GET(minIndex) > HEAPCOST_GET(rightChildIndex))) {
		minIndex = rightChildIndex;
	}

	if (minIndex != index) {
		//need swap
		T temp = heapVector[index];
		heapVector[index] = heapVector[minIndex];
		HEAP_SET_ELEMENT_POSITION(heapVector[index], index);
		heapVector[minIndex] = temp;
		HEAP_SET_ELEMENT_POSITION(heapVector[minIndex], minIndex);
		BubbleDown(minIndex);
	}

}

template<class T>
void Heap<T>::BubbleUp(int index) {
	if (index == 0)
		return;

	int parentIndex = PARENT(index);		//(index - 1) / 2;
	if ( HEAPCOST_GET(parentIndex) > HEAPCOST_GET(index)) {
		T temp = heapVector[parentIndex];
		heapVector[parentIndex] = heapVector[index];
		HEAP_SET_ELEMENT_POSITION(heapVector[parentIndex], parentIndex);
		heapVector[index] = temp;
		HEAP_SET_ELEMENT_POSITION(heapVector[index], index);
		BubbleUp(parentIndex);
	}
}

template<class T>
void Heap<T>::push(T newValue) {
	int newIndex = heapVector.size();
	heapVector.push_back(newValue);
	HEAP_SET_ELEMENT_POSITION(newValue, newIndex);
	BubbleUp(newIndex);
}

template<class T>
T Heap<T>::pop() {
	T min = NULL;
	int size = heapVector.size();
	//WVARIABLE(size);
	if (size > 0) {
		min = this->get();
		//WINFO("pos before "<<HEAP_GET_ELEMENT_POSITION(heapVector[size - 1]));
		heapVector[0] = heapVector[size - 1];
		HEAP_SET_ELEMENT_POSITION(heapVector[0], 0);
		//WINFO("pos "<<HEAP_GET_ELEMENT_POSITION(heapVector[0]));
		heapVector.pop_back();
		BubbleDown(0);
	} else {
		ERROR("empty heap");
	}

	return min;
}

template<class T>
T Heap<T>::get() {
	return heapVector[0];
}

template<class T>
T Heap<T>::get(int id) {
	return heapVector[id];
}

template<class T>
void Heap<T>::replace(int id, T newValue) {
	double oldCost = HEAPCOST_GET(id);
	double newCost = newValue->getValue();
	heapVector[id] = newValue;
	HEAP_SET_ELEMENT_POSITION(heapVector[id], id);
	if (newCost < oldCost) {
		//zmenseni hodnoty -- chce to jit nahoru
		BubbleUp(id);
	} else {
		BubbleDown(id);
	}
}

template<class T>
int Heap<T>::size() {
	return heapVector.size();
}

template<class T>
void Heap<T>::updateCost(int position, double cost) {
	double oldCost = HEAPCOST_GET(position);
	HEAPCOST_SET(position, cost);
	if (oldCost > cost) {
		//zmenseni hodnoty -- chce to jit nahoru
		BubbleUp(position);
	} else {
		BubbleDown(position);
	}
}

template<class T>
void Heap<T>::updateCost(T value, double cost) {
	int size = heapVector.size();
	//INFO("value->heap_position "<<value->heap_position);
	updateCost(value->heap_position, cost);

	/*
	 INFO("bad updateCost begin");
	 for (int var = 0; var < size; ++var) {
	 ERROR("bad update cost "<<var)
	 if (heapVector[var] == value) {
	 INFO("found right heap at pos "<<var)
	 INFO("heap position "<<value->heap_position);
	 updateCost(var, cost);
	 break;
	 }
	 }
	 INFO("bad updateCost end");
	 */
}

template<class T>
void Heap<T>::clear() {
	heapVector.clear();
}

template<class T>
bool Heap<T>::empty() {
	return this->heapVector.empty();
}

template<class T>
std::vector<T> Heap<T>::getHeapVector() {
	return this->heapVector;
}

template<class T>
bool Heap<T>::checkOrdering() {
	checkIndex(0);
}

template<class T>
bool Heap<T>::checkIndex(int index) {
	if (index < heapVector.size()) {
		int lchild = LCHILD(index);
		int rchild = RCHILD(index);
		if (lchild < heapVector.size()) {
			if ( HEAPCOST_GET(lchild) < HEAPCOST_GET(index)) {
				ERROR("heap inconsistency lchild" <<lchild);
				exit(1);
			}
			checkIndex(lchild);
		}
		if (rchild < heapVector.size()) {
			if ( HEAPCOST_GET(rchild) < HEAPCOST_GET(index)) {
				ERROR("heap inconsistency rchild" <<rchild);
				exit(1);
			}
			checkIndex(rchild);
		}
	}
}
#endif /* SRC_PLANNER_HEAP_H_ */
