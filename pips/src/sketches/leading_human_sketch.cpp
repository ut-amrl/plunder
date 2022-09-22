#include <memory>

#include "../ast.hpp"
#include "../enumeration.hpp"

using namespace AST;
using namespace std;

extern "C" void* get_sketch() {
  Param human_thresh("human_thresh", {2, 0, 0}, NUM);
  Param door_thresh("door_thresh", {2, 0, 0}, NUM);
  Param door_state_p("door_state_p", {0, 0, 0}, NUM);
  Feature human_distance("human_distance", {2, 0, 0}, NUM);
  Feature door_distance("door_distance", {2, 0, 0}, NUM);
  Feature door_state_f("door_state_f", {0, 0, 0}, NUM);

  /*
  if EucDist(robot_pos, human_pos) > 1 || (EucDist(robot_pos, door_pos) < 2 &&
  door_state != OPEN (0)) { state = STOP } else { state = GO
  }
  */
  std::shared_ptr<BinOp> cond = make_shared<BinOp>(
      make_shared<BinOp>(make_shared<Feature>(human_distance),
                         make_shared<Param>(human_thresh), "Gt"),
      make_shared<BinOp>(
          make_shared<BinOp>(make_shared<Feature>(door_distance),
                             // make_shared<Num>(Num(2.0f, {2, 0, 0})),
                             make_shared<Param>(door_thresh), "Lt"),
          make_shared<UnOp>(
              make_shared<BinOp>(make_shared<Feature>(door_state_f),
                                 make_shared<Param>(door_state_p),
                                 // make_shared<Num>(Num(0.0f, {0,0,0})),
                                 "Eq"),
              "Not"),
          "And"),
      "Or");

  const Sketch* sketch = new Sketch{make_pair(cond, SymEntry(0.0f))};
  return (void*)sketch;
}