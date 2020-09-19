/*

Project: bilateral chain realization for keyframe 
Date: 20/08/05
@Author: Wang
Detail: Data Structure for the further need
Scenario: mapping

*/

#include <iostream>
#include <cmath>
#include <stdlib.h>


using namespace std;
 
//node
class Node {
public:
	char data;
	Node *pPre, *pNext;
};
 
//Chain
class DoubleCircularLinkList {
public:
	DoubleCircularLinkList() {
		head = new Node;
		head->data = 0;
		head->pNext = head;
		head->pPre = head;
	}// initialization 

	~DoubleCircularLinkList() {delete head;} //why by this way?
	void CreateLinkList(int n);
	void InsertNode(int position, int d);
	void TraverseLinkList();
	bool IsEmpty();
	int GetLength();
	void DeleteNode(int posiiton);
	void DeleteLinkList();
private:
	Node *head;
};


//Func: import nodes into the chain
void DoubleCircularLinkList::CreateLinkList(int n) {
	if (n < 0) {
		cout << "Import node number not appropriate!" << endl;
		exit(EXIT_FAILURE);
	}
	else {
		int i = 0;
		Node *pnew, *ptemp;
		ptemp = head;
		i = n;
 
		while (n-- > 0) {
			pnew = new Node;
			cout << "Now importing the No." << i - n << "node...";	
			cin >> pnew->data;
			pnew->pNext = head;
			pnew->pPre = ptemp;
			ptemp->pNext = pnew;
			ptemp = pnew;
		}
	}
}


//Func: Insert 
void DoubleCircularLinkList::InsertNode(int position, int d) {
	if (position < 0 || position > GetLength() + 1) {
		cout << "Wrong index to Insert!" << endl;
		exit(EXIT_FAILURE);
	}
	else {
		Node *pnew, *ptemp;
		pnew = new Node;
		pnew->data = d;
		ptemp = head;
		while (position-- > 1)
			ptemp = ptemp->pNext;
		pnew->pNext = ptemp->pNext;
		pnew->pPre = ptemp;
		ptemp->pNext = pnew;
		ptemp = pnew;
	}
}
 
void DoubleCircularLinkList::TraverseLinkList() {
	Node *ptemp = head->pNext;
	while(ptemp != head) {
		cout << ptemp->data << " ";
		ptemp = ptemp->pNext;
	}
	cout << endl;
}
 
bool DoubleCircularLinkList::IsEmpty() {
	if (head == head->pNext)
		return true;
	else
		return false;
}
 
int DoubleCircularLinkList::GetLength() {
	int n = 0;
	Node *ptemp = head->pNext;
	while (ptemp != head) {
		n++;
		ptemp = ptemp->pNext;
	}
	return n;
}
 
void DoubleCircularLinkList::DeleteNode(int position) {
	if (position < 0 || position > GetLength()) {
		cout << "Wrong index to delete" << endl;
		exit(EXIT_FAILURE);
	}
	else {
		Node *pdelete, *ptemp;
		ptemp = head;
		while (position-- > 1)
			ptemp = ptemp->pNext;
		pdelete = ptemp->pNext;
		ptemp->pNext = pdelete->pNext;
		pdelete->pNext->pPre = ptemp;
		delete pdelete;
		pdelete = NULL;
	}
}
 
void DoubleCircularLinkList::DeleteLinkList() {
	Node *pdelete, *ptemp;
	pdelete = head->pNext;
	while (pdelete != head) {
		ptemp = pdelete->pNext;
		head->pNext = ptemp;
		ptemp->pPre = head;
		delete pdelete;
		pdelete = ptemp;
	}
}
 
int main() {
	DoubleCircularLinkList dcl;
	int position = 0, value = 0, n = 0;
	bool flag = false;
 
	cout << "the number of node in the chain you wanna create:";
	cin >> n;
	dcl.CreateLinkList(n);
 
	cout << "the chain is:";
	dcl.TraverseLinkList();
 
	cout << "please type the index and data you wanna insert:";
	cin >> position >> value;
	dcl.InsertNode(position, value);
	
	cout << "the chain is:";
	dcl.TraverseLinkList();
 
	cout << "please type the index of the node you wanna delete:";
	cin >> position;
	dcl.DeleteNode(position);
 
	cout << "the chain is:";
	dcl.TraverseLinkList();
 
	dcl.DeleteLinkList();
	flag = dcl.IsEmpty();
	if (flag)
		cout << "success" << endl;
	else
		cout << "fail" << endl;
 
 	return 0;
}
