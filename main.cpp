// ESE-549 Project 3 Reference Solution
// Updating for Spring 2018
// Peter Milder

/** @file */

// For you to do:

// Part 1: 
//    - Write the simFullCircuit() function (based on your project 2 code)
//    - Write the getObjective() function
//    - Write the updateDFrontier() function
//    - Write the backtrace() function
//    - Write the podemRecursion() function
// Then your basic PODEM implementation should be finished.
// Test this code carefully.
// Note there is also a "checkTest" function (enabled by default) that 
// will use your simulator to run the test after you generated it and 
// check that it correctly detects the faults.
//
// Part 2:
//    - Write the eventDrivenSim() function.
//    - Change the simulation calls in your podemRecursion() function
//      from using the old simFullCircuit() function to the new
//      event-driven simulator when mode == 2.
//      (Make sure your original code still works correctly when 
//       mode == 1.)
// Then, your PODEM implementation should run considerably faster
// (probably 8 to 10x faster for large circuits).
// Test everything and then evaluate the speedup.
// A quick way to figure out how long it takes to run something in
// Linux is:
//   time ./atpg ... type other params...
//
// Part 3:
//    - Write code to perform equivalence fault collapsing
//    - Test and evaluate your result
//
// Part 4:
//    - Apply optimizations as discussed in project description

// I have marked several places with TODO for you.

#include <iostream>
#include <fstream> 
#include <vector>
#include <queue>
#include <time.h>
#include <stdio.h>
#include "parse_bench.tab.h"
#include "ClassCircuit.h"
#include "ClassGate.h"
#include "ClassFaultEquiv.h"
#include <limits>
#include <stdlib.h>
#include <time.h>

using namespace std;

/**  @brief Just for the parser. Don't touch. */
extern "C" int yyparse();

/**  Input file for parser. Don't touch. */
extern FILE *yyin;

/** Our circuit. We declare this external so the parser can use it more easily. */
extern Circuit* myCircuit;

//--------------------------
// Helper functions
void printUsage();
bool checkTest(Circuit* myCircuit);
string printPIValue(char v);
void setValueCheckFault(Gate* g, char gateValue);
//--------------------------

//----------------------------
// Functions for logic simulation
void simFullCircuit(Circuit* myCircuit);
void eventDrivenSim(Circuit* myCircuit, queue<Gate*> q);
//----------------------------

//----------------------------
// Functions for PODEM:
bool podemRecursion(Circuit* myCircuit);
bool getObjective(Gate* &g, char &v, Circuit* myCircuit);
void updateDFrontier(Circuit* myCircuit);
void backtrace(Gate* &pi, char &piVal, Gate* objGate, char objVal, Circuit* myCircuit);
//--------------------------
void andNandGateEqual(int gateType,vector<Gate*> &gatePredessors,Gate* gate,FaultEquiv &myFaultEquivGraph);
void orNorGateEqual(int gateType,vector<Gate*> &gatePredessors,Gate* gate,FaultEquiv &myFaultEquivGraph);
void notGateEqual(vector<Gate*> &gatePredessors,Gate* gate,FaultEquiv &myFaultEquivGraph);
void setEquivalent(Gate* gate,FaultEquiv &myFaultEquivGraph);

//----------------
// Please put the prototypes for functions you add here.
//added by xuechun xie, for Part2 and Part3
int gateDepth(Gate* gate);
void setGateDepth(Gate* gate);
int finalDepth=1;
int max(int value1,int value2);
int simulation(Gate* gate);
int get_ExpOutput(int pos0,int pos1,int posx,int posD,int posB,int gateType);
int set_ActualOutput(Gate* gate,int expectedOutput);
int andGateOutput(int pos0,int posx,int pos1,int posD,int posB);
int nandGateOutput(int pos0,int posx,int pos1,int posD,int posB);
int orGateOutput(int pos0,int posx,int pos1,int posD,int posB);
int norGateOutput(int pos0,int posx,int pos1,int posD,int posB);
int notGateOutput(int pos0,int posx,int pos1,int posD,int posB);
int xorGateOutput(int pos0,int posx,int pos1,int posD,int posB);
int xnorGateOutput(int pos0,int posx,int pos1,int posD,int posB);
int buffGateOutput(int pos0,int posx,int pos1,int posD,int posB);
//-----

///////////////////////////////////////////////////////////
// Global variables
// These are made global to make your life slightly easier.

/** Global variable: a vector of Gate pointers for storing the D-Frontier. */
vector<Gate*> dFrontier;

/** Global variable: holds a pointer to the gate with stuck-at fault on its output location. */
Gate* faultLocation;     

/** Global variable: holds the logic value you will need to activate the stuck-at fault. */
char faultActivationVal;

/** Global variable: which part of the project are you running? */
int mode = -1;


/** @brief The main function.
 * 
 * You do not need to change anything in the main function,
 * although you should understand what is happening
 * here.
 */
int main(int argc, char* argv[]) {

	// Check the command line input and usage
	if (argc != 5) {
		printUsage();    
		return 1;
	}
	
	mode = atoi(argv[1]);
	if ((mode < 1) || (mode > 5)) {
		printUsage();    
		return 1;   
	}

	// Parse the bench file and initialize the circuit.
	FILE *benchFile = fopen(argv[2], "r");
	if (benchFile == NULL) {
		cout << "ERROR: Cannot read file " << argv[2] << " for input" << endl;
		return 1;
	}
	yyin=benchFile;
	yyparse();
	fclose(benchFile);

	myCircuit->setupCircuit(); 
	cout << endl;

	// Setup the output text files
	ofstream outputStream, equivStream;
	string dotOutFile = argv[4];
	dotOutFile += ".out";
	outputStream.open(dotOutFile);
	if (!outputStream.is_open()) {
		cout << "ERROR: Cannot open file " << dotOutFile << " for output" << endl;
		return 1;
	}
	if (mode >= 3) {
		string dotFCFile = argv[4];
		dotFCFile += ".fc";
		equivStream.open(dotFCFile);
		if (!equivStream.is_open()) {
			cout << "ERROR: Cannot open file " << dotFCFile << " for output" << endl;
			return 1;
		}

	}
	 
	// Open the fault file.
	ifstream faultStream;
	string faultLocStr;
	faultStream.open(argv[3]);
	if (!faultStream.is_open()) {
		cout << "ERROR: Cannot open fault file " << argv[3] << " for input" << endl;
		return 1;
	}
	
	// This vector will hold all of the faults you want to generate tests for. 
	// For Parts 1 and 2, you will simply go through this vector in order. 
	// For Part 3, you will start from this list and apply equivalence fault collapsing.
	// For Part 4, you can try other techniques.
	vector<faultStruct> faultList;


	// This code goes through each line in the fault file and adds the faults to 
	// faultList.
	while(getline(faultStream, faultLocStr)) {
		string faultTypeStr;
	myCircuit->clearGateValues();
      myCircuit->setPIValues(constructInputLine(inputLine));
      
      /////////////////////////////////////////////////////////////////////////////
      // Write your code here.
      // At this point, the system has set up the data structure (like in Proj. 1) and 
      // set the input values for the PI gates (like in Proj. 1).
      // It has additionally used the new capability for placing faults on gate outputs
      // to set the fault in the appropriate gate.
      // add by xuechun xie, to simulate gates output
      vector<Gate*> MyCircuitPOs = myCircuit->getPOGates();
     for(int i=0; i<MyCircuitPOs.size(); i++){
      int output=simulation(MyCircuitPOs[i]);
      //set_ActualOutput(MyCircuitPOs[i],output);
      }
     
      // Stop writing your code here.
      ////////////////////////////////////////////////////
      
			
		if (!(getline(faultStream, faultTypeStr))) {
			break;
		}
			
		char faultType = atoi(faultTypeStr.c_str());
		Gate* fl = myCircuit->findGateByName(faultLocStr);

		faultStruct fs = {fl, faultType};
		faultList.push_back(fs);
	}
	faultStream.close();


	// --------- Equivalence Fault Collapsing (Part 3) ---------------
	// For Part 3 and Part 4, you will do equivalence based
	// fault collapsing here. 

	// The FaultEquiv hold the fault collapsing graph. (See description below
	// and in project handout.)
	FaultEquiv myFaultEquivGraph;

	vector<faultStruct> origFaultList;
	if (mode >= 3) {

		// Keep the uncollapsed list here. You can use this to test your Part 3 code later.
		origFaultList = faultList;

		// Create a new FaultEquiv, which stores the Fault Equivalence
		// relationships in your circuit.
		myFaultEquivGraph.init(faultList);

		// -------------- your Part 3 code starts here ----------------
		// TODO

		// Perform equivalence-based fault collapsing here. (You probably want to
		// write and call one or more other functions here.

		// The basic idea behind he FaultEquiv data structure:
		//   - A FaultEquiv holds a vector of faultEquivNode.
		//   - Each faultEquivNode holds three things:
		//        (1) a vector of equivalent faults
		//        (2) a vector of pointers to other faultEquivNodes that this one 
		//            dominates
		//        (3) a vector of pointers to other faultEquivNodes that are 
		//            dominated by this node

		// For Part 3, you will just use equivalence. (Later, one of the options 
		// for Part 4 will be to exploit dominance also.)

		// When you initialize a FaultEquiv, it takes faultList (the list of all
		// faults you are currently considering, which comes from the .fault file you
		// supply at the command line). Then, it initializes the structure so that
		// each fault gets its own faultEquivNode, with no equivalence or dominance set up.

		// Then, your goal is to go through the circuit, and based on the topology and
		// gate types, tell the FaultEquiv structure which nodes to set as equivalent.
		// (And later, you can choose to do the same with dominance in Part 4 if you 
		//  want.)

		// Start by reading the documentation and looking through ClassFaultEquiv.h and .cc.

		// --------- Example -----------------------
		// Example for working with FaultEquiv:
		//    Imagine that g is a Gate* pointing to a 2-input AND gate.
		//    Therefore you know that the output and both inputs of that 
		//    gate SA0 are equivalent.
		//    You could then run:

		//       Gate* a = g->get_gateInputs()[0];
		//       Gate* b = g->get_gateInputs()[1];
		//       FaultEquiv.mergeFaultEquivNodes(a, FAULT_SA0, b, FAULT_SA0);
		//       FaultEquiv.mergeFaultEquivNodes(a, FAULT_SA0, g, FAULT_SA0);

		//    Each time you run mergeFaultEquivNodes, the FaultEquiv structure
		//    looks for nodes that match the specified faults (e.g. a sa0 and b sa0),
		//    and then merges them into a single node.

		//    You can run myFaultEquivGraph.printFaultEquiv(cout); which will 
		//    print the status of the fault equivalence node. When you are 
		//    getting started, I recommend starting with a very small circuit 
		//    and use the print function frequently. Follow along by hand.
	
		// If you want to print the current state of the FaultEquiv, run this:
		//    myFaultEquivGraph.printFaultEquiv(cout);

		// Your algorithm should walk through each gate, and figure out which
		// faults to make equivalent. Note that if you get to a "fanout" gate,
		// *none of its faults are equivalent!*


		// One other possibly helpful note: I added a public bool value called 
		// "visited" to ClassGate. When a gate is created, visited is set to 
		// false. You may use this to make sure when you are performing fault 
		// equivalence, that you don't include the same gate multiple times. 
		// When you look at a Gate, check that visited == false. Then, after 
		// you perform fault collapsing on that gate, set visited to true.
		// (This may or may not help, depending on how you choose to structure 
		// your fault equivalence code.)

		//2018-04-10,xuechun xie,added to go through the circuit, and set equivalent
		vector<Gate*> MyCircuitPOs = myCircuit->getPOGates();
     	for(int i=0; i<MyCircuitPOs.size(); i++){
      		setEquivalent(MyCircuitPOs[i],myFaultEquivGraph);
      	}


		// end of your equivalence fault collapsing code
		/////////////////////////////////////////////////

		// This will print the fault equivalence result to the .fc output file
		myFaultEquivGraph.printFaultEquiv(equivStream);
		equivStream.close();

		// Update the faultList to now be the reduced list
		// Your PODEM code will now find tests only for the collapsed faults.
		faultList = myFaultEquivGraph.getCollapsedFaultList();

		// Just for your information
		cout << "Original list length: " << origFaultList.size() << endl;
		cout << "Collapsed length: " << faultList.size() << endl;

	}
	// ------------- End of Equivalence Fault Collapsing -------------


	// ------------------------- Part 4 -----------------------------
	if (mode == 4) {
		// TODO
		// You can put mode == 4 code here if you want
		// (or maybe you will just include these optimizations inside of your
		// main PODEM code, and you can remove this)


	}
	if (mode == 5) {
		// TODO
		// Here you should start your code for mode 5, test set size reduction


	}
	// -----------End of Part 4 ---------------------------------


	// ------------- PODEM code ----------------------------------
	// This will run in all parts by default. 

	// We will use this to keep track of any undetectable faults. This may
	// be useful for you depending on what you do in Part 4.
	vector<faultStruct> undetectableFaults;

	// We will use this to store all the tests that your algorithm
	// finds. You may want to use this in checking correctness of
	// your program.
	vector<vector<char>> allTests;


	// This is the main loop that performs PODEM.
	// It iterates over all faults in the faultList. For each one,
	// it will set up the fault and call your podemRecursion() function.
	// You should not have to change this, but you should understand how 
	// it works.
	for (int faultNum = 0; faultNum < faultList.size(); faultNum++) {

		// Clear the old fault in your circuit, if any.
		myCircuit->clearFaults();
		 
		// Set up the fault we are trying to detect
		faultLocation  = faultList[faultNum].loc;
		char faultType = faultList[faultNum].val;
		faultLocation->set_faultType(faultType);      
		faultActivationVal = (faultType == FAULT_SA0) ? LOGIC_ONE : LOGIC_ZERO;
		 
		// Set all gate values to X
		for (int i=0; i < myCircuit->getNumberGates(); i++) {
			myCircuit->getGate(i)->setValue(LOGIC_X);
		}

		// initialize the D frontier.
		dFrontier.clear();
		
		// call PODEM recursion function
		bool res = podemRecursion(myCircuit);

		// If we succeed, print the test we found to the output file, and 
		// store the test in the allTests vector.
		if (res == true) {
			vector<Gate*> piGates = myCircuit->getPIGates();
			vector<char> thisTest;
			for (int i=0; i < piGates.size(); i++) {
				// Print PI value to output file
				outputStream << printPIValue(piGates[i]->getValue());

				// Store PI value for later
				char v = piGates[i]->getValue();
				if (v == LOGIC_D)
					v = LOGIC_ONE;
				else if (v == LOGIC_DBAR)
					v = LOGIC_ZERO;
				thisTest.push_back(v);			

			}
			outputStream << endl;
			allTests.push_back(thisTest);
		}

		// If we failed to find a test, print a message to the output file
		else 
			outputStream << "none found" << endl;
		
		// Lastly, you can use this to test that your PODEM-generated test
		// correctly detects the already-set fault.
		// Of course, this assumes that your simulation code is correct.
		// Comment this out when you are evaluating the runtime of your
		// ATPG system because it will add extra time.
		if (res == true) {
			if (!checkTest(myCircuit)) {
				cout << "ERROR: PODEM returned true, but generated test does not detect this fault on PO." << endl;
				//myCircuit->printAllGates(); // uncomment if you want to see what is going on here
				assert(false);
			}
		}
		
		// Just printing to screen to let you monitor progress. You can comment this
		// out if you like.
		cout << "Fault = " << faultLocation->get_outputName() << " / " << (int)(faultType) << ";";
		if (res == true) 
			cout << " test found; " << endl;
		else {
			cout << " no test found; " << endl;
			faultStruct f = {faultLocation, faultType};
			undetectableFaults.push_back(f);
		}
	}

	// clean up and close the output stream
	delete myCircuit;
	outputStream.close();

	return 0;
}


/////////////////////////////////////////////////////////////////////
// Functions in this section are helper functions.
// You should not need to change these, except if you want
// to enable the checkTest function (which will use your simulator
// to attempt to check the test vector computed by PODEM.)


/** @brief Print usage information (if user provides incorrect input).
 * 
 * You don't need to touch this.
 */
void printUsage() {
	cout << "Usage: ./atpg [mode] [bench_file] [fault_file] [output_base]" << endl << endl;
	cout << "   mode:        1 through 5" << endl;
	cout << "   bench_file:  the target circuit in .bench format" << endl;
	cout << "   fault_file:  faults to be considered" << endl;
	cout << "   output_base: basename for output file" << endl;
	cout << endl;
	cout << "   The system will generate a test pattern for each fault listed" << endl;
	cout << "   in fault_file and store the result in output_loc.out" << endl;
	cout << "   If you are running Part 3 or 4, it will also print the result" << endl;
	cout << "   of your equivalence fault collapsing in file output_loc.fc" << endl << endl;
	cout << "   Example: ./atpg 3 test/c17.bench test/c17.fault myc17" << endl;
	cout << "      --> This will run your Part 3 code and produce two output files:" << endl;
	cout << "          1: myc17.out - contains the test vectors you generated for these faults" << endl;
	cout << "          2: myc17.fc  - contains the FaultEquiv data structure that shows your" << endl;
	cout << "                         equivalence classes" << endl;
	cout << endl;	
}


/** @brief Uses *your* simulator to check validity of your test.
 * 
 * This function can be called after your PODEM algorithm finishes.
 * If you enable this, it will clear the circuit's internal values,
 * and re-simulate the vector PODEM found to test your result.
 
 * This is helpful when you are developing and debugging, but will just
 * slow things down once you know things are correct.
 
 * Important: this function of course assumes that your simulation code 
 * is correct. If your simulation code is incorrect, then this is not
 * helpful to you.
*/
bool checkTest(Circuit* myCircuit) {

	simFullCircuit(myCircuit);

	// look for D or D' on an output
	vector<Gate*> poGates = myCircuit->getPOGates();
	for (int i=0; i<poGates.size(); i++) {
		char v = poGates[i]->getValue();
		if ((v == LOGIC_D) || (v == LOGIC_DBAR)) {
			return true;
		}
	}

	// If we didn't find D or D' on any PO, then our test was not successful.
	return false;

}

/** @brief Prints a PI value. 
 * 
 * This is just a helper function used when storing the final test you computed.
 *  You don't need to run or modify this.
 */
string printPIValue(char v) {
	switch(v) {
		case LOGIC_ZERO: return "0";
		case LOGIC_ONE: return "1";
		case LOGIC_UNSET: return "U";
		case LOGIC_X: return "X";
		case LOGIC_D: return "1";
		case LOGIC_DBAR: return "0";
	}
	return "";
}

/** @brief Set the value of Gate* g to value gateValue, accounting for any fault on g.
		\note You will not need to modify this.
 */
void setValueCheckFault(Gate* g, char gateValue) {
	if ((g->get_faultType() == FAULT_SA0) && (gateValue == LOGIC_ONE)) 
		g->setValue(LOGIC_D);
	else if ((g->get_faultType() == FAULT_SA0) && (gateValue == LOGIC_DBAR)) 
		g->setValue(LOGIC_ZERO);
	else if ((g->get_faultType() == FAULT_SA1) && (gateValue == LOGIC_ZERO)) 
		g->setValue(LOGIC_DBAR);
	else if ((g->get_faultType() == FAULT_SA1) && (gateValue == LOGIC_D)) 
		g->setValue(LOGIC_ONE);
	else
		g->setValue(gateValue);
}

// end of helper functions
//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
// Start of functions for circuit simulation.

/** @brief Runs full circuit simulation
 *
 * Adapt your Project 2 code for this!
 */
void simFullCircuit(Circuit* myCircuit) {

	// Clear all the non-PI gate values
	for (int i=0; i<myCircuit->getNumberGates(); i++) {
		Gate* g = myCircuit->getGate(i);
		if (g->get_gateType() != GATE_PI)
			g->setValue(LOGIC_UNSET);      
	}  

	// TODO
	// Write your simulation code here. Use your Project 2 code.
	// Also: don't forget to check for faults on the PIs; you either
	// need to do this here *or* in the PODEM recursion when you set
	// the value.

}

/** @brief Perform event-driven simulation.
 * \note You will write this function in Part 2.
 * 
 * Please see the project handout for a description of what
 * we are doing here and why.

 * This function takes as input the Circuit* and a queue<Gate*>
 * indicating the remaining gates that need to be evaluated.
 */
void eventDrivenSim(Circuit* myCircuit, queue<Gate*> q) {
	
	// TODO

}

// End of functions for circuit simulation
////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////
// Begin functions for PODEM.

/** @brief PODEM recursion.
 *
 * \note For Part 1, you must write this, following the pseudocode from class. 
 * Make use of the getObjective and backtrace functions.
 * For Part 2, you will add code to this that calls your eventDrivenSim() function.
 * For Parts 3 and 4, use the eventDrivenSim() version.
 */
bool podemRecursion(Circuit* myCircuit) {

	// TODO

	// Write the PODEM recursion function here. It should:
	//   - Check for D or D' on any PO and return true if so
	//   - Call getObjective
	//   - Call backtrace
	//   - Set the PI indicated by your backtrace function.
	//       - Be careful you are doing the right thing if there is a fault on a PI
	//   - Recurse
	//   - If recursion succeeds, return true.
	//   - If recursion fails, reverse value you set on the PI and recurse again.
  	//   - If recursion succeeds, return true.
  	//   - If neither recursive call returns true, imply the PI = X and return false



	return false;
}

// Find the objective for myCircuit. The objective is stored in g, v.
// 
// class or your textbook.
/** @brief PODEM objective function.
 *  \param g Use this pointer to store the objective Gate your function picks.
 *  \param v Use this char to store the objective value your function picks.
 *  \param myCircuit A pointer to the Circuit being considered
 *  \returns True if the function is able to determine an objective, and false if it fails.
 * \note For Part 1, you must write this, following the pseudocode in class and the code's comments.
 */
bool getObjective(Gate* &g, char &v, Circuit* myCircuit) {

	// TODO

	// Note: you have a global variable Gate* faultLocation which represents
	// the gate with the fault. Use this when you need to check if the 
	// fault has been excited yet.

	// Another note: if the fault is not excited but the fault 
	// location value is not X, then we have failed to activate 
	// the fault. In this case getObjective should fail and Return false.  
	// Otherwise, use the D-frontier to find an objective.

	// Don't forget to call updateDFrontier() to make sure your dFrontier 
	// (a global variable) is up to date.

	// Remember, for Parts 1/2 if you want to match my reference solution exactly,
	// you should choose the first gate in the D frontier (dFrontier[0]), and pick
	// its first approrpriate input.
	


	return true;
}


// A very simple method to update the D frontier.
/** @brief A simple method to compute the set of gates on the D frontier.
 *
 * \note For Part 1, you must write this. The simplest form follows the pseudocode included in the comments.
 */

void updateDFrontier(Circuit* myCircuit) {

	// TODO

	// Procedure:
	//  - clear the dFrontier vector (stored as the global variable dFrontier -- see the 
	//    top of the file)
	//  - loop over all gates in the circuit; for each gate, check if it should be on 
	//    D-frontier; if it is, add it to the dFrontier vector.

	// If you want to match my output exactly, go through the gates in this order:
	//     for (int i=0; i < myCircuit->getNumberGates(); i++) {    
	//         Gate* g = myCircuit->getGate(i);

	// One way to improve speed for Part 4 would be to improve D-frontier management. 
	// You can add/remove gates from the D frontier during simulation, instead of adding 
	// an entire pass over all the gates like this.


}


// Backtrace: given objective objGate and objVal, then figure out which input (pi) to set 
// and which value (piVal) to set it to.

/** @brief PODEM backtrace function
 * \param pi Output: A Gate pointer to the primary input your backtrace function found.
 * \param piVal Output: The value you want to set that primary input to
 * \param objGate Input: The objective Gate (computed by getObjective)
 * \param objVal Input: the objective value (computed by getObjective)
 * \param myCircuit Input: pointer to the Circuit being considered
 * \note Write this function based on the psuedocode from class.
 */
void backtrace(Gate* &pi, char &piVal, Gate* objGate, char objVal, Circuit* myCircuit) {

	// TODO

	// Write your backtrace code here.

	// If you want your solution to match mine exactly:
	//   1. Go through the gate inputs in order; pick the first X input.
	//   2. Treat XOR gates like they are OR and XNOR like they are NOR.

}

////////////////////////////////////////////////////////////////////////////

// Please place any functions you add here, between these bars.
//added by xuechun xie, for part2
int gateDepth(Gate* gate){
    vector<Gate*> gatePredessors=gate->get_gateInputs();  
    int initalDepth=0;
    int preDepth=-1;
    if(gatePredessors.size()==0){   
      return 0;
    }else{
            for(int i=0;i<gatePredessors.size();i++){
              //changed by xuechun xie to avoid recalculate depth if it is set
              preDepth=gatePredessors[i]->getDepth();
              // preDepth==-1 means the depth is unset,otherwise set
              if(preDepth==-1){
                initalDepth=max(initalDepth,gateDepth(gatePredessors[i]));
              }else{
                initalDepth=max(initalDepth,preDepth);
              }
            }
            gate->setDepth(initalDepth+1);
            return initalDepth+1;
          }
}

int max(int value1,int value2){
  if(value1>value2){
    return value1;
  }else{
    return value2;
  }
}
//........................................................................
//codes for part3, 
int simulation(Gate* gate){
    vector<Gate*> gatePredessors=gate->get_gateInputs();
    int expectedOutput;
    int actualOutput;
    //int actualInput[gatePredessors.size()];
    int actualInput;
    int gateType;
    int pos0=-1;
    int posx=-1;
    int pos1=-1;
    int posD=-1;
    int posB=-1;
    int gateValue=LOGIC_UNSET;
    if(gatePredessors.size()==0){
      expectedOutput=gate->getValue();
      return expectedOutput;
    }else{
            for(int i=0;i<gatePredessors.size();i++){
            //iterate each input value of the gate, and check if 0,1,X occurs in the input
            //20180301,changed to avoid redundant set value
            //if gate value is unset, then set it, else return the gate value                         
              //actualInput=simulation(gatePredessors[i]);
              gateValue=gatePredessors[i]->getValue();
              if(gateValue==LOGIC_UNSET){
                actualInput=simulation(gatePredessors[i]);          
              }else{
                actualInput=gateValue;
              }
              if(actualInput==0)
                pos0=1;
              else if(actualInput==4)
                posx=1;
              else if(actualInput==1)
                pos1=1;
              //20180301,changed to add type of D(2) and D'(3)
              else if(actualInput==2)
                posD=1;
              else if(actualInput==3)
                posB=1;                       
            }
            gateType=gate->get_gateType();
            //geting each gate's output according to gate type and input controling value            
            expectedOutput=get_ExpOutput(pos0,pos1,posx,posD,posB,gateType);
            actualOutput=set_ActualOutput(gate,expectedOutput);
            return actualOutput; 
          }
    }  

//return gate's output according to gatetype and input value 0,1,X  
int get_ExpOutput(int pos0,int pos1,int posx,int posD,int posB,int gateType){
    switch(gateType) {      
    case GATE_NAND: return nandGateOutput(pos0,posx,pos1,posD,posB);
    case GATE_AND: return andGateOutput(pos0,posx,pos1,posD,posB);
    case GATE_OR: return orGateOutput(pos0,posx,pos1,posD,posB);
    case GATE_NOR: return norGateOutput(pos0,posx,pos1,posD,posB);
    case GATE_NOT: return notGateOutput(pos0,posx,pos1,posD,posB);
    case GATE_XOR: return xorGateOutput(pos0,posx,pos1,posD,posB);
    case GATE_XNOR: return xnorGateOutput(pos0,posx,pos1,posD,posB);
    case GATE_BUFF: return buffGateOutput(pos0,posx,pos1,posD,posB);
    case GATE_FANOUT: return buffGateOutput(pos0,posx,pos1,posD,posB);     
    default: return -1;
    }            
}

int andGateOutput(int pos0,int posx,int pos1,int posD,int posB){
     if((pos0==1)||(posD==1&&posB==1))
        return LOGIC_ZERO;
      else if(posx==1)
        return LOGIC_X;
      else if(posD==1)
        return LOGIC_D;
      else if(posB==1)
        return LOGIC_DBAR;      
      else
        return LOGIC_ONE;
}

int nandGateOutput(int pos0,int posx,int pos1,int posD,int posB){
      if((pos0==1)||(posD==1&&posB==1))
        return LOGIC_ONE;      
      else if(posx==1)
        return LOGIC_X;
      else if(posD==1)
        return LOGIC_DBAR; 
      else if(posB==1)
        return LOGIC_D;       
      else
        return LOGIC_ZERO;
}

int orGateOutput(int pos0,int posx,int pos1,int posD,int posB){
      if((pos1==1)||(posD==1&&posB==1))
        return LOGIC_ONE;      
      else if(posx==1)
        return LOGIC_X;
      else if(posD==1)
        return LOGIC_D;                                     
      else if(posB==1)
        return LOGIC_DBAR;
      else
        return LOGIC_ZERO; 
}

int norGateOutput(int pos0,int posx,int pos1,int posD,int posB){
      if((pos1==1)||(posD==1&&posB==1))
        return LOGIC_ZERO;      
      else if(posx==1)
        return LOGIC_X;
      else if(posD==1)
        return LOGIC_DBAR;
      else if(posB==1)
        return LOGIC_D;      
      else
        return LOGIC_ONE;
} 

int notGateOutput(int pos0,int posx,int pos1,int posD,int posB){
      if(pos1==1)
        return LOGIC_ZERO;
      else if(posx==1)      
        return LOGIC_X;
      else if(posD==1)
        return LOGIC_DBAR;
      else if(posB==1)
        return LOGIC_D;  
      else
        return LOGIC_ONE;
}

int xorGateOutput(int pos0,int posx,int pos1,int posD,int posB){
      if((pos1==1&&pos0==1)||(posD==1&&posB==1))
        return LOGIC_ONE;
      else if(posx==1)
        return LOGIC_X;
      else if((posD==1&&pos0==1)||(posB==1&&pos1==1))
        return LOGIC_D;
      else if((posD==1&&pos1==1)||(posB==1&&pos0==1))
        return LOGIC_DBAR;      
      else    
        return LOGIC_ZERO;
}

int xnorGateOutput(int pos0,int posx,int pos1,int posD,int posB){
      if((pos1==1&&pos0==1)||(posD==1&&posB==1))
        return LOGIC_ZERO;
      else if(posx==1)
        return LOGIC_X;
      else if((posD==1&&pos0==1)||(posB==1&&pos1==1))
        return LOGIC_DBAR;
      else if((posD==1&&pos1==1)||(posB==1&&pos0==1))
        return LOGIC_D;      
      else 
        return LOGIC_ONE;      
}

int buffGateOutput(int pos0,int posx,int pos1,int posD,int posB){
      if(pos1==1)
        return LOGIC_ONE;
      else if(posx==1)
        return LOGIC_X;
      else if(posD==1)
        return LOGIC_D;
      else if(posB==1)
        return LOGIC_DBAR;        
      else 
        return LOGIC_ZERO;
}

int set_ActualOutput(Gate* gate,int expectedOutput){
  int gateFaultType;
  gateFaultType=gate->get_faultType();
  switch(gateFaultType){
    case NOFAULT:
      gate->setValue(expectedOutput);
      return expectedOutput;
    case FAULT_SA0:
      if(expectedOutput==0||expectedOutput==LOGIC_X){
        gate->setValue(expectedOutput);
        return expectedOutput;
      }else{
        gate->setValue(LOGIC_D);
        return LOGIC_D;         
      }          
    case FAULT_SA1:
      if(expectedOutput==1||expectedOutput==LOGIC_X){
        gate->setValue(expectedOutput);
        return expectedOutput;
      }else{
        gate->setValue(LOGIC_DBAR);
        return LOGIC_DBAR;        
      }     
  }
}
///////////////////////////////////////////////////////////////
// Please place any new functions you add here, between these two bars.
//2018.04.11,xuechun xie, add functions for part3: fault equivalent
////////////////////////////////////////////////////////////////
void setEquivalent(Gate* gate,FaultEquiv &myFaultEquivGraph){
	int gateType;
	bool valVisited=false;	
	vector<Gate*> gatePredessors=gate->get_gateInputs();

    if(gatePredessors.size()==0){
      return ;
    }else{
    		
            for(int i=0;i<gatePredessors.size();i++){
            	valVisited=gatePredessors[i]->visited;
            	cout<<valVisited<<endl;
            	if(valVisited==false){
              		//gateType=gatePredessors[i]->get_gateType();
                	setEquivalent(gatePredessors[i],myFaultEquivGraph);          
              	}                   
            }
            gateType=gate->get_gateType();
            cout<<"gatetype:"<<gateType<<endl;
            switch(gateType) {      
		    case GATE_NAND: case GATE_AND: { 
		    	andNandGateEqual(gateType,gatePredessors,gate,myFaultEquivGraph);
				gate->visited=true;
				cout<<"andNandGateEqual："<<gate->visited<<endl;
				return ;
			}
		    case GATE_OR: case GATE_NOR:{
		    	orNorGateEqual(gateType,gatePredessors,gate,myFaultEquivGraph);
		    	gate->visited=true;
		    	cout<<"orNorGateEqual："<<gate->visited<<endl;
				return ;

		    } 
		    case GATE_NOT: {
		    	notGateEqual(gatePredessors,gate,myFaultEquivGraph);
		    	gate->visited=true;
		    	cout<<"NotGateEqual："<<gate->visited<<endl;
				return ;
		    } 
		    default: {
		    	gate->visited=true;
		    	cout<<"default: "<<gate->visited<<endl;
		    	return ;
			}
		}
            
        }
          
}

void andNandGateEqual(int gateType,vector<Gate*> &gatePredessors,Gate* gate,FaultEquiv &myFaultEquivGraph){
	int j;
	for(int i=0;i<gatePredessors.size()-1;i++){
		j=i+1;
		myFaultEquivGraph.mergeFaultEquivNodes(gatePredessors[i+1], FAULT_SA0, gatePredessors[i], FAULT_SA0);
	}
	if(gateType==GATE_AND){
		myFaultEquivGraph.mergeFaultEquivNodes(gate, FAULT_SA0,gatePredessors[j], FAULT_SA0);
	}else if(gateType==GATE_NAND){
		myFaultEquivGraph.mergeFaultEquivNodes(gate, FAULT_SA1,gatePredessors[j], FAULT_SA0);
	}
	return ;
}

void orNorGateEqual(int gateType,vector<Gate*> &gatePredessors,Gate* gate,FaultEquiv &myFaultEquivGraph){
	int j;
	for(int i=0;i<gatePredessors.size()-1;i++){
		j=i+1;
		myFaultEquivGraph.mergeFaultEquivNodes(gatePredessors[i], FAULT_SA1, gatePredessors[i+1], FAULT_SA1);
	}
	if(gateType==GATE_OR){
		myFaultEquivGraph.mergeFaultEquivNodes(gate, FAULT_SA1,gatePredessors[0], FAULT_SA1);
	}else if(gateType==GATE_NOR){
		myFaultEquivGraph.mergeFaultEquivNodes(gate, FAULT_SA0,gatePredessors[0], FAULT_SA1);
	}
	return ;
}
 
 void notGateEqual(vector<Gate*> &gatePredessors,Gate* gate,FaultEquiv &myFaultEquivGraph){
	myFaultEquivGraph.mergeFaultEquivNodes(gate, FAULT_SA0,gatePredessors[0], FAULT_SA1);
	myFaultEquivGraph.mergeFaultEquivNodes(gate, FAULT_SA1,gatePredessors[0], FAULT_SA0);
	return ;
}

////////////////////////////////////////////////////////////////////////////


