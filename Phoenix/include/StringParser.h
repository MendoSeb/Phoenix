#include <vector>
#include <string>


namespace StringParser
{
	struct Node
	{
		std::string operation;
		Node* c1;
		Node* c2;
	};


	Node* ParseString(std::string& expression);

}