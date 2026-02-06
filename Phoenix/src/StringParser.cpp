#include "StringParser.h"
#include <cassert>


namespace StringParser
{
	Node* ParseString(std::string& e)
	{
		char type[100];
		char rest[100];
		char value1[100];
		char value2[100];

		int res1 = sscanf_s(e.c_str(), "%99[^(](%999[^)]", &type, 100, &rest, 100);
		int res2 = sscanf_s(rest, "%s", &value1, 100, &value2, 100);
		int res3 = sscanf_s(rest, "%s, %s", &value1, 100, &value2, 100);

		/*std::string type_s = type;
		std::string rest_s = rest;

		if 
		char value[100];

		// on arrive sur un noeud final
		int res2 = sscanf_s(e.c_str(), "%s", &value, 100);
		Node* n = new Node();
		n->operation = value;*/
	
		return nullptr;
	}
}