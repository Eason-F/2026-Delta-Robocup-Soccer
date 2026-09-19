#include <util/StartingPreset.hpp>
#include <cassert>
#include <iostream>
uint32_t testMillis = 1000;
static void initialize(StartingPreset &p) {
    for (int &pin : testPins) pin = HIGH;
    p.setup();
}
static void edge(StartingPreset &p, int pin, bool pressed, bool running=false) {
    testPins[pin] = pressed ? LOW : HIGH;
    p.update(running); testMillis += 25; p.update(running);
}
static void click(StartingPreset &p, int pin, bool running=false) {
    edge(p,pin,true,running); edge(p,pin,false,running);
}
static void single(StartingPreset &p, int pin) {
    click(p,pin); testMillis += 500; assert(p.update(false));
}
static void position(const StartingPreset &p, float x, float y) {
    assert(p.hasPosition()); assert(p.getPosition().x == x && p.getPosition().y == y);
}
int main() {
    StartingPreset p; initialize(p);
    assert(!p.hasPosition() && !p.bothAttack());
    // No bounce event; a press must be stable for 25 ms.
    testPins[12]=LOW; p.update(false); testMillis+=10;
    testPins[12]=HIGH; p.update(false); testMillis+=600; p.update(false);
    assert(!p.hasPosition());
    click(p,12); testMillis+=474; p.update(false); assert(!p.hasPosition());
    ++testMillis; assert(p.update(false)); position(p,0,-150);
    single(p,13); position(p,0,-615);
    single(p,14); position(p,0,-815);
    click(p,13); testMillis+=100; click(p,13); position(p,-450,-615);
    click(p,14); testMillis+=100; click(p,14); position(p,450,-615);
    click(p,12); click(p,12); assert(p.bothAttack()); position(p,450,-615);
    single(p,12); assert(!p.bothAttack()); position(p,0,-150);
    // Latest button wins; expiry of an older pending click cannot override it.
    click(p,13); click(p,14); testMillis+=500; p.update(false); position(p,0,-815);
    p.update(true); click(p,13,true); testMillis+=600; p.update(true); position(p,0,-815);
    assert(p.update(false)); assert(!p.hasPosition() && !p.bothAttack());
    p.update(true); p.update(false); assert(!p.hasPosition());
    // Starting during a pending click waits out the window and locks new input.
    click(p,13); assert(!p.update(true)); assert(p.hasPendingPress());
    click(p,13,true); // An attempted second press after start is ignored.
    testMillis+=500; assert(p.update(true)); position(p,0,-615);
    assert(!p.hasPendingPress());
    // A held gameplay button does not become a press on entering reset.
    edge(p,14,true,true); p.update(false); testMillis+=600; p.update(false);
    assert(!p.hasPosition()); edge(p,14,false); single(p,14); position(p,0,-815);
    // Holding does not generate repeats or a double press.
    edge(p,13,true); testMillis+=500; p.update(false); position(p,0,-615);
    testMillis+=1000; p.update(false); position(p,0,-615); edge(p,13,false);
    // 500 ms boundary and millis() wrap use unsigned elapsed time.
    p.update(true); p.update(false);
    click(p,13); testMillis+=450; edge(p,13,true); position(p,-450,-615);
    edge(p,13,false);
    p.update(true); p.update(false); testMillis=0xffffff00u;
    click(p,14); testMillis+=500; p.update(false); position(p,0,-815);
    std::cout << "Preset tests passed: mapping, debounce, timing, latest selection, gameplay lock and reset.\n";
}
