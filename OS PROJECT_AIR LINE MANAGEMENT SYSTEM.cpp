#include <iostream>
#include <string>
#include <vector>
#include <queue>
#include <ctime>
#include <cstdlib>
#include <thread>
#include <mutex>
#include <chrono>
#include <atomic>
#include <iomanip>
#include <limits>
#include <SFML/Graphics.hpp>
using namespace std;
using namespace chrono;

enum AircraftType { COMMERCIAL, CARGO, EMERGENCY };
enum FlightPhase { HOLDING, APPROACH, LANDING, TAXI, AT_GATE, TAKEOFF_ROLL, CLIMB, CRUISE };
enum Direction { NORTH, SOUTH, EAST, WEST, ARR, DEP };
enum RunwayID { RWY_A, RWY_B, RWY_C };
enum FlightStatus { WAITING, ACTIVE, COMPLETED };

const int NorthTime = 180; // every 3 min
const int SouthTime = 120; // every 2 min
const int EastTime = 150; // every 2.5 min
const int WestTime = 240; // every 4 min
const int SIMULATION_DURATION = 300; // 5 minutes
const float FUEL_THRESHOLD = 100.0f;
typedef system_clock Clock;
typedef duration<double> Duration;

std::string formatTime(time_t t) {
    struct tm now_tm;
    localtime_r(&t, &now_tm);
    std::ostringstream oss;
    oss << std::put_time(&now_tm, "%H:%M:%S");
    return oss.str();
}

string getDirectionMeaning(Direction dir) {
    switch (dir) {
        case NORTH: return "International Arrival (NORTH)";
        case SOUTH: return "Domestic Arrival (SOUTH)";
        case EAST:  return "International Departure (EAST)";
        case WEST:  return "Domestic Departure (WEST)";
        case ARR:   return "Emergency Arrival (ARR)";
        case DEP:   return "Emergency Departure (DEP)";
        default:    return "Unknown Direction";
    }
}

string getAircraftTypeName(AircraftType type) {
    switch (type) {
        case COMMERCIAL: return "Commercial";
        case CARGO: return "Cargo";
        case EMERGENCY: return "Emergency";
        default: return "Unknown";
    }
}

struct Airline {
    string name;
    AircraftType type;
    int totalAircraft;
    int activeFlights;
    int avnCount = 0;

    Airline(string n, AircraftType t, int aircraft, int flights)
        : name(n), type(t), totalAircraft(aircraft), activeFlights(flights) {}
};

vector<Airline> airlines = {
    Airline("PIA", COMMERCIAL, 6, 4),
    Airline("AirBlue", COMMERCIAL, 4, 4),
    Airline("FedEx", CARGO, 3, 2),
    Airline("PAF", EMERGENCY, 2, 1),
    Airline("Blue Dart", CARGO, 2, 2),
    Airline("AghaKhan Air", EMERGENCY, 2, 1)
};

class Runway;

class Flight {
private:
    float calculateFuelConsumption() const {
        switch (phase) {
            case HOLDING: return 1.2f;
            case APPROACH: return 1.0f;
            case LANDING: return 1.0f;
            case TAXI: return 0.5f;
            case AT_GATE: return 0.4f;
            case TAKEOFF_ROLL: case CLIMB: return 2.0f;
            default: return 1.0f;
        }
    }

public:
    string flightNumber;
    Airline* airline;
    Direction direction;
    AircraftType aircraftType;
    FlightPhase phase;
    int speed;
    FlightStatus status;
    time_t scheduledTime;
    bool hasViolation;
    Runway* assignedRunway;
    float fuel;
    float max_fuel_capacity;
    time_t last_update_time;
    time_t queueEntryTime;
    time_t phaseStartTime;

    Flight(string fn, Airline* al, Direction dir, AircraftType atype, time_t sched)
        : flightNumber(fn), airline(al), direction(dir), aircraftType(atype), scheduledTime(sched),
          assignedRunway(nullptr), hasViolation(false), status(WAITING) {
        speed = isArrival() ? (400 + (rand() % 201)) : 0;
        queueEntryTime = time(nullptr);
        switch (atype) {
            case COMMERCIAL: fuel = 12000.0f; break;
            case CARGO: fuel = 15000.0f; break;
            case EMERGENCY: fuel = 10000.0f; break;
        }
        max_fuel_capacity = fuel;
        last_update_time = time(nullptr);
        phase = isArrival() ? HOLDING : AT_GATE;
        phaseStartTime = time(nullptr);
    }

    float generateRandomSpeed(FlightPhase phase, bool allowViolation = true) {
        bool generateViolation = allowViolation && (rand() % 100) < 20;
        switch (phase) {
            case HOLDING:
                return generateViolation ? ((rand() % 2) ? 350 + rand() % 50 : 600 + rand() % 100) : 400 + rand() % 201;
            case APPROACH:
                return generateViolation ? ((rand() % 2) ? 200 + rand() % 40 : 290 + rand() % 50) : 240 + rand() % 51;
            case LANDING: {
                float progress = static_cast<float>(time(nullptr) - phaseStartTime) / getPhaseDuration();
                float baseSpeed = 240 - (210 * progress);
                float variation = (rand() % 21) - 10;
                return max(0.0f, baseSpeed + variation);
            }
            case TAXI:
                return generateViolation ? ((rand() % 2) ? 5 + rand() % 10 : 30 + rand() % 20) : 15 + rand() % 16;
            case AT_GATE:
                return generateViolation ? 5 + rand() % 10 : rand() % 6;
            case TAKEOFF_ROLL:
                return 200 + rand() % 91;
            case CLIMB:
                return 300 + rand() % 164;
            case CRUISE:
                return 800 + rand() % 101;
            default:
                return 0;
        }
    }

    bool isArrival() const { return direction == NORTH || direction == SOUTH || direction == ARR; }
    bool isDeparture() const { return direction == EAST || direction == WEST || direction == DEP; }

    void updateFuel() {
        float consumption = calculateFuelConsumption();
        time_t now = time(nullptr);
        float elapsed = difftime(now, last_update_time);
        fuel -= consumption * elapsed;
        last_update_time = now;
        float newSpeed = generateRandomSpeed(phase);
        updateSpeed(newSpeed);
        if (fuel < FUEL_THRESHOLD && aircraftType != EMERGENCY) {
            aircraftType = EMERGENCY;
            direction = isArrival() ? ARR : DEP;
            cout << "EMERGENCY: " << flightNumber << " low fuel!\n";
        }
    }

    void updateSpeed(float newSpeed) {
        speed = newSpeed;
        checkViolation();
    }

    void checkViolation() {
        bool violation = false;
        float permissibleSpeed = 0.0f;
        switch (phase) {
            case HOLDING:
                permissibleSpeed = 600.0f;
                if (speed < 400 || speed > 600) {
                    violation = true;
                    cout << "Violation: Flight " << flightNumber << " speed " << speed << " km/h in HOLDING (400–600) [AVN issued]\n";
                }
                break;
            case APPROACH:
                permissibleSpeed = 290.0f;
                if (speed < 240 || speed > 290) {
                    violation = true;
                    cout << "Violation: Flight " << flightNumber << " speed " << speed << " km/h in APPROACH (240–290) [AVN issued]\n";
                }
                break;
            case LANDING:
                permissibleSpeed = 240.0f;
                if (speed > 240 || speed < 30) {
                    violation = true;
                    cout << "Violation: Flight " << flightNumber << " speed " << speed << " km/h in LANDING (30–240) [AVN issued]\n";
                }
                break;
            case TAXI:
                permissibleSpeed = 30.0f;
                if (speed > 30) {
                    violation = true;
                    cout << "Violation: Flight " << flightNumber << " speed " << speed << " km/h in TAXI (≤30) [AVN issued]\n";
                }
                break;
            case AT_GATE:
                permissibleSpeed = 10.0f;
                if (speed > 10) {
                    violation = true;
                    cout << "Violation: Flight " << flightNumber << " speed " << speed << " km/h at GATE (≤10) [AVN issued]\n";
                }
                break;
            case TAKEOFF_ROLL:
                permissibleSpeed = 290.0f;
                if (speed > 290) {
                    violation = true;
                    cout << "Violation: Flight " << flightNumber << " speed " << speed << " km/h in TAKEOFF_ROLL (≤290) [AVN issued]\n";
                }
                break;
            case CLIMB:
                permissibleSpeed = 463.0f;
                if (speed > 463) {
                    violation = true;
                    cout << "Violation: Flight " << flightNumber << " speed " << speed << " km/h in CLIMB (≤463) [AVN issued]\n";
                }
                break;
            case CRUISE:
                permissibleSpeed = 900.0f;
                if (speed < 800 || speed > 900) {
                    violation = true;
                    cout << "Violation: Flight " << flightNumber << " speed " << speed << " km/h in CRUISE (800–900) [AVN issued]\n";
                }
                break;
        }
        if (violation && !hasViolation) {
            hasViolation = true;
            airline->avnCount++;
        }
    }

    void progressPhase() {
        if (status == COMPLETED) return;
        if (isArrival()) {
            switch (phase) {
                case HOLDING: phase = APPROACH; break;
                case APPROACH: phase = LANDING; break;
                case LANDING: phase = TAXI; break;
                case TAXI: phase = AT_GATE; break;
                case AT_GATE: status = COMPLETED; break;
                default: break;
            }
        } else {
            switch (phase) {
                case AT_GATE: phase = TAXI; break;
                case TAXI: phase = TAKEOFF_ROLL; break;
                case TAKEOFF_ROLL: phase = CLIMB; break;
                case CLIMB: phase = CRUISE; status = COMPLETED; break;
                default: break;
            }
        }
        speed = generateRandomSpeed(phase, false);
        phaseStartTime = time(nullptr);
    }

    void assignRunway(Runway* rwy) { assignedRunway = rwy; }

    void setStatus(FlightStatus newStatus) { status = newStatus; }

    int getPhaseDuration() const {
        switch (phase) {
            case HOLDING: return 10;
            case APPROACH: return 8;
            case LANDING: return 10;
            case TAXI: return 8;
            case AT_GATE: return 10;
            case TAKEOFF_ROLL: return 8;
            case CLIMB: return 10;
            default: return 0;
        }
    }

    int getWaitTime() const { return difftime(time(nullptr), queueEntryTime); }
    int getPriority() const {
        if (aircraftType == EMERGENCY) return 0;
        if (aircraftType == CARGO) return 1;
        return 2;
    }

    // Getters
    const string& getFlightNumber() const { return flightNumber; }
    Airline* getAirline() const { return airline; }
    Direction getDirection() const { return direction; }
    AircraftType getAircraftType() const { return aircraftType; }
    FlightPhase getPhase() const { return phase; }
    float getSpeed() const { return speed; }
    FlightStatus getStatus() const { return status; }
    time_t getScheduledTime() const { return scheduledTime; }
    bool getHasViolation() const { return hasViolation; }
    Runway* getAssignedRunway() const { return assignedRunway; }
    float getFuel() const { return fuel; }
};

class Runway {
private:
    RunwayID id;
    mutex runwayMutex;
    atomic<bool> occupied;
    Flight* currentFlight;
    Clock::time_point occupiedSince;
    std::thread runwayThread;
    std::atomic<bool> running{false};

    void runwayThreadLoop() {
        while (running) {
            {
                std::lock_guard<std::mutex> lock(runwayMutex);
                if (occupied && currentFlight) {
                    auto now = Clock::now();
                    double elapsed = Duration(now - occupiedSince).count();
                    if (elapsed >= 10.0) {
                        std::cout << "[Runway " << id << "] Forced release after 10s by "
                                  << currentFlight->getFlightNumber() << std::endl;
                        occupied = false;
                        currentFlight = nullptr;
                    }
                }
            }
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

public:
    Runway(RunwayID runwayId)
        : id(runwayId), occupied(false), currentFlight(nullptr) {
        running = true;
        runwayThread = std::thread(&Runway::runwayThreadLoop, this);
    }
    ~Runway() {
        running = false;
        if (runwayThread.joinable()) {
            runwayThread.join();
        }
    }

    Clock::time_point getOccupiedSince() const { return occupiedSince; }

    bool tryOccupy(Flight* flight) {
        lock_guard<mutex> lock(runwayMutex);
        if (!occupied) {
            occupied = true;
            currentFlight = flight;
            occupiedSince = Clock::now();
            return true;
        }
        auto now = Clock::now();
        if (Duration(now - occupiedSince).count() >= 10.0) {
            occupied = true;
            currentFlight = flight;
            occupiedSince = now;
            return true;
        }
        return false;
    }

    void releaseRunway() {
        lock_guard<mutex> lock(runwayMutex);
        occupied = false;
        currentFlight = nullptr;
    }

    RunwayID getID() const { return id; }
    bool isOccupied() const { return occupied; }
    Flight* getFlight() const { return currentFlight; }
};

struct AVN {
    string avnID;
    string flightNumber;
    string airlineName;
    AircraftType aircraftType;
    float recordedSpeed;
    float permissibleSpeed;
    time_t issuanceTime;
    double fineAmount;
    string paymentStatus;
    time_t dueDate;

    AVN(string id, Flight* flight, float permSpeed)
        : avnID(id), flightNumber(flight->getFlightNumber()), airlineName(flight->getAirline()->name),
          aircraftType(flight->getAircraftType()), recordedSpeed(flight->getSpeed()),
          permissibleSpeed(permSpeed), issuanceTime(time(nullptr)), paymentStatus("unpaid") {
        fineAmount = (aircraftType == COMMERCIAL ? 500000 : 700000) * 1.15; // 15% service fee
        dueDate = issuanceTime + 3 * 24 * 3600; // 3 days
    }
};

class AirTrafficControl {
private:
    Runway* rwyA;
    Runway* rwyB;
    Runway* rwyC;
    mutex queueMutex;
    vector<AVN> avns;

    struct EmergencyComparator {
        bool operator()(Flight* a, Flight* b) {
            return a->getScheduledTime() > b->getScheduledTime();
        }
    };

    struct PriorityQueue {
        priority_queue<Flight*, vector<Flight*>, EmergencyComparator> emergency;
        queue<Flight*> cargo;
        queue<Flight*> commercial;
    };

    PriorityQueue arrivalQueue;
    PriorityQueue departureQueue;

public:
    AirTrafficControl(Runway* a, Runway* b, Runway* c) : rwyA(a), rwyB(b), rwyC(c) {}

    Runway* getRunwayA() const { return rwyA; }
    Runway* getRunwayB() const { return rwyB; }
    Runway* getRunwayC() const { return rwyC; }

    void addFlight(Flight* flight) {
        lock_guard<mutex> lock(queueMutex);
        if (flight->isArrival()) {
            if (flight->getAircraftType() == EMERGENCY) {
                arrivalQueue.emergency.push(flight);
                cout << "[DEBUG] Queued Emergency Arrival Flight " << flight->getFlightNumber() << "\n";
            } else if (flight->getAircraftType() == CARGO) {
                arrivalQueue.cargo.push(flight);
            } else {
                arrivalQueue.commercial.push(flight);
            }
        } else {
            if (flight->getAircraftType() == EMERGENCY) {
                departureQueue.emergency.push(flight);
                cout << "[DEBUG] Queued Emergency Departure Flight " << flight->getFlightNumber() << "\n";
            } else if (flight->getAircraftType() == CARGO) {
                departureQueue.cargo.push(flight);
            } else {
                departureQueue.commercial.push(flight);
            }
        }
    }

    void scheduleFlights() {
        lock_guard<mutex> lock(queueMutex);
        processQueue(arrivalQueue, true);
        processQueue(departureQueue, false);
    }

    void processQueue(PriorityQueue& queue, bool isArrival) {
        while (!queue.emergency.empty()) {
            Flight* flight = queue.emergency.top();
            if (assignRunway(flight, isArrival)) {
                cout << "[DEBUG] Assigned runway to Emergency Flight " << flight->getFlightNumber() << "\n";
                queue.emergency.pop();
            } else {
                break;
            }
        }
        size_t cargoCount = queue.cargo.size();
        for (size_t i = 0; i < cargoCount; ++i) {
            if (queue.cargo.empty()) break;
            Flight* flight = queue.cargo.front();
            if (assignRunway(flight, isArrival)) {
                cout << "[DEBUG] Assigned runway to Cargo Flight " << flight->getFlightNumber() << "\n";
                queue.cargo.pop();
            } else {
                queue.cargo.pop();
                queue.cargo.push(flight);
            }
        }
        size_t commCount = queue.commercial.size();
        for (size_t i = 0; i < commCount; ++i) {
            if (queue.commercial.empty()) break;
            Flight* flight = queue.commercial.front();
            if (assignRunway(flight, isArrival)) {
                cout << "[DEBUG] Assigned runway to Commercial Flight " << flight->getFlightNumber() << "\n";
                queue.commercial.pop();
            } else {
                queue.cargo.pop();
                queue.cargo.push(flight);
            }
        }
    }

    bool assignRunway(Flight* flight, bool isArrival) {
        Runway* target = nullptr;
        if (flight->getAircraftType() == EMERGENCY) {
            target = rwyC;
            if (target->tryOccupy(flight)) {
                flight->assignRunway(target);
                flight->progressPhase();
                cout << "[FR2.1/FR2.3] Emergency Flight " << flight->getFlightNumber()
                     << " assigned to " << runwayToString(target->getID()) << "\n";
                return true;
            }
            target = isArrival ? rwyA : rwyB;
            if (target->tryOccupy(flight)) {
                flight->assignRunway(target);
                flight->progressPhase();
                cout << "[FR2.3] Emergency Flight " << flight->getFlightNumber()
                     << " assigned to backup " << runwayToString(target->getID()) << "\n";
                return true;
            }
        }
        if (flight->getAircraftType() == CARGO) {
            target = rwyC;
            if (target->tryOccupy(flight)) {
                flight->assignRunway(target);
                cout << "[FR2.1] Cargo Flight " << flight->getFlightNumber()
                     << " assigned to " << runwayToString(target->getID()) << "\n";
                return true;
            }
            Flight* current = target->getFlight();
            if (current && current->getAircraftType() == COMMERCIAL) {
                cout << "[FR2.1] Preempting Commercial Flight " << current->getFlightNumber()
                     << " with Cargo Flight " << flight->getFlightNumber() << "\n";
                target->releaseRunway();
                addFlight(current);
                if (target->tryOccupy(flight)) {
                    flight->assignRunway(target);
                    cout << "[FR2.1] Cargo Flight " << flight->getFlightNumber()
                         << " assigned to " << runwayToString(target->getID()) << "\n";
                    return true;
                }
            }
        }
        if (flight->getAircraftType() == COMMERCIAL) {
            target = isArrival ? rwyA : rwyB;
            if (target->tryOccupy(flight)) {
                flight->assignRunway(target);
                cout << "[FR2.1] Commercial Flight " << flight->getFlightNumber()
                     << " assigned to " << runwayToString(target->getID()) << "\n";
                return true;
            }
            target = rwyC;
            if (!target->isOccupied() && target->tryOccupy(flight)) {
                flight->assignRunway(target);
                cout << "[FR2.3] Commercial Flight " << flight->getFlightNumber()
                     << " assigned to backup/overflow " << runwayToString(target->getID()) << "\n";
                return true;
            }
        }
        return false;
    }

    void generateAVN(Flight* flight, float permissibleSpeed) {
        string avnID = "AVN" + to_string(avns.size() + 1);
        avns.emplace_back(avnID, flight, permissibleSpeed);
        cout << "AVN Generated: ID=" << avnID << ", Flight=" << flight->getFlightNumber()
             << ", Fine=" << avns.back().fineAmount << " PKR\n";
    }

    void displayAVNs() {
        cout << "\n=== Active AVNs ===\n";
        for (const auto& avn : avns) {
            cout << "AVN ID: " << avn.avnID << ", Flight: " << avn.flightNumber
                 << ", Airline: " << avn.airlineName << ", Type: " << getAircraftTypeName(avn.aircraftType)
                 << ", Speed: " << avn.recordedSpeed << " vs " << avn.permissibleSpeed
                 << ", Fine: " << avn.fineAmount << " PKR, Status: " << avn.paymentStatus
                 << ", Due: " << formatTime(avn.dueDate) << "\n";
        }
    }

    bool payAVN(const string& avnID, double amount) {
        for (auto& avn : avns) {
            if (avn.avnID == avnID && avn.paymentStatus == "unpaid" && amount >= avn.fineAmount) {
                avn.paymentStatus = "paid";
                cout << "Payment Successful for AVN " << avnID << "\n";
                return true;
            }
        }
        cout << "Payment Failed for AVN " << avnID << "\n";
        return false;
    }

    string runwayToString(RunwayID id) {
        switch (id) {
            case RWY_A: return "RWY_A";
            case RWY_B: return "RWY_B";
            case RWY_C: return "RWY_C";
            default: return "UNKNOWN";
        }
    }

    void displayQueues() {
        lock_guard<mutex> lock(queueMutex);
        auto showQueueDetails = [](const PriorityQueue& q, const string& type) {
            cout << "\n=== " << type << " QUEUE ===\n";
            cout << "Emergency (" << q.emergency.size() << "):\n";
            auto emerCopy = q.emergency;
            while (!emerCopy.empty()) {
                Flight* f = emerCopy.top();
                cout << " - " << f->getFlightNumber() << " (Sched: " << formatTime(f->getScheduledTime()) << ")\n";
                emerCopy.pop();
            }
            cout << "Cargo (" << q.cargo.size() << "):\n";
            auto cargoCopy = q.cargo;
            while (!cargoCopy.empty()) {
                Flight* f = cargoCopy.front();
                cout << " - " << f->getFlightNumber() << " (Sched: " << formatTime(f->getScheduledTime()) << ")\n";
                cargoCopy.pop();
            }
            cout << "Commercial (" << q.commercial.size() << "):\n";
            auto commCopy = q.commercial;
            while (!commCopy.empty()) {
                Flight* f = commCopy.front();
                cout << " - " << f->getFlightNumber() << " (Sched: " << formatTime(f->getScheduledTime()) << ")\n";
                commCopy.pop();
            }
        };
        showQueueDetails(arrivalQueue, "ARRIVAL");
        showQueueDetails(departureQueue, "DEPARTURE");
    }
};

class Simulation {
private:
    AirTrafficControl* atc;
    vector<Flight*> pending, active;
    int duration = SIMULATION_DURATION;
    time_t simulationStartTime;
    sf::RenderWindow* window;

public:
    Simulation(AirTrafficControl* atcController, sf::RenderWindow* win)
        : atc(atcController), window(win) {}

    void addFlight(Flight* f) { pending.push_back(f); }

    string runwayToString(RunwayID id) {
        switch (id) {
            case RWY_A: return "RWY_A";
            case RWY_B: return "RWY_B";
            case RWY_C: return "RWY_C";
            default: return "UNKNOWN";
        }
    }

    string flightPhaseToString(FlightPhase phase) {
        switch (phase) {
            case HOLDING: return "HOLDING";
            case APPROACH: return "APPROACH";
            case LANDING: return "LANDING";
            case TAXI: return "TAXI";
            case AT_GATE: return "AT_GATE";
            case TAKEOFF_ROLL: return "TAKEOFF_ROLL";
            case CLIMB: return "CLIMB";
            case CRUISE: return "CRUISE";
            default: return "UNKNOWN";
        }
    }

    string flightStatusToString(FlightStatus status) {
        switch (status) {
            case WAITING: return "WAITING";
            case ACTIVE: return "ACTIVE";
            case COMPLETED: return "COMPLETED";
            default: return "UNKNOWN";
        }
    }

    bool randomGroundFault() { return (rand() % 100) < 5; }

    void displayFlights() {
        cout << "\n=== Flight States ===\n";
        cout << left << setw(12) << "Flight" << setw(15) << "Airline"
             << setw(12) << "Type" << setw(15) << "Phase"
             << setw(10) << "Speed" << setw(12) << "Status"
             << setw(15) << "Runway" << endl;
        for (Flight* f : active) {
            string runway = f->getAssignedRunway() ? runwayToString(f->getAssignedRunway()->getID()) : "None";
            cout << left << setw(12) << f->getFlightNumber()
                 << setw(15) << f->getAirline()->name
                 << setw(12) << getAircraftTypeName(f->getAircraftType())
                 << setw(15) << flightPhaseToString(f->getPhase())
                 << setw(10) << f->getSpeed()
                 << setw(12) << flightStatusToString(f->getStatus())
                 << setw(15) << runway << endl;
        }
    }

    void displayRunwaysStatus() {
        cout << "\n=== Runway Statuses ===\n";
        auto showRunwayState = [this](Runway* rwy) {
            cout << runwayToString(rwy->getID()) << ": "
                 << (rwy->isOccupied() ? "Occupied by " + rwy->getFlight()->getFlightNumber() : "Available") << endl;
        };
        showRunwayState(atc->getRunwayA());
        showRunwayState(atc->getRunwayB());
        showRunwayState(atc->getRunwayC());
    }

    void renderVisualization() {
        window->clear(sf::Color::Black);
        sf::Font font;
        if (!font.loadFromFile("arial.ttf")) {
            cout << "Error loading font\n";
            return;
        }

        // Draw runways
        sf::RectangleShape rwyA(sf::Vector2f(300, 50));
        rwyA.setPosition(100, 100);
        rwyA.setFillColor(atc->getRunwayA()->isOccupied() ? sf::Color::Red : sf::Color::Green);
        window->draw(rwyA);
        sf::Text rwyAText("RWY_A", font, 20);
        rwyAText.setPosition(100, 80);
        window->draw(rwyAText);

        sf::RectangleShape rwyB(sf::Vector2f(300, 50));
        rwyB.setPosition(100, 200);
        rwyB.setFillColor(atc->getRunwayB()->isOccupied() ? sf::Color::Red : sf::Color::Green);
        window->draw(rwyB);
        sf::Text rwyBText("RWY_B", font, 20);
        rwyBText.setPosition(100, 180);
        window->draw(rwyBText);

        sf::RectangleShape rwyC(sf::Vector2f(300, 50));
        rwyC.setPosition(100, 300);
        rwyC.setFillColor(atc->getRunwayC()->isOccupied() ? sf::Color::Red : sf::Color::Green);
        window->draw(rwyC);
        sf::Text rwyCText("RWY_C", font, 20);
        rwyCText.setPosition(100, 280);
        window->draw(rwyCText);

        // Draw ATC tower
        sf::RectangleShape tower(sf::Vector2f(50, 100));
        tower.setPosition(450, 150);
        tower.setFillColor(sf::Color::Blue);
        window->draw(tower);
        sf::Text towerText("ATC", font, 20);
        towerText.setPosition(450, 130);
        window->draw(towerText);

        // Draw aircraft
        for (const auto* flight : active) {
            sf::CircleShape aircraft(10);
            aircraft.setFillColor(flight->getAircraftType() == EMERGENCY ? sf::Color::Yellow :
                                  flight->getAircraftType() == CARGO ? sf::Color::Magenta : sf::Color::Cyan);
            float x = 100 + (flight->getPhase() * 50);
            float y = flight->getAssignedRunway() ? (flight->getAssignedRunway()->getID() == RWY_A ? 110 :
                                                     flight->getAssignedRunway()->getID() == RWY_B ? 210 : 310) : 400;
            aircraft.setPosition(x, y);
            window->draw(aircraft);
            sf::Text flightText(flight->getFlightNumber(), font, 15);
            flightText.setPosition(x, y - 20);
            window->draw(flightText);
        }

        window->display();
    }

    void start() {
        simulationStartTime = time(nullptr);
        cout << "Simulation starts at: " << formatTime(simulationStartTime) << "\n";
        auto begin = Clock::now();
        while (Duration(Clock::now() - begin).count() < duration && window->isOpen()) {
            sf::Event event;
            while (window->pollEvent(event)) {
                if (event.type == sf::Event::Closed) window->close();
            }

            cout << "[DEBUG] Simulation tick at " << formatTime(time(nullptr)) << "\n";
            time_t now = time(nullptr);

            // Check runway timeouts
            for (Runway* rwy : {atc->getRunwayA(), atc->getRunwayB(), atc->getRunwayC()}) {
                if (rwy->isOccupied()) {
                    auto elapsed = Duration(Clock::now() - rwy->getOccupiedSince()).count();
                    if (elapsed >= 10.0) {
                        cout << "[DEBUG] Releasing runway " << runwayToString(rwy->getID())
                             << " for flight " << rwy->getFlight()->getFlightNumber() << "\n";
                        rwy->releaseRunway();
                    }
                }
            }

            // Move flights from pending to active
            for (auto it = pending.begin(); it != pending.end();) {
                if ((*it)->scheduledTime <= now) {
                    (*it)->setStatus(ACTIVE);
                    atc->addFlight(*it);
                    active.push_back(*it);
                    it = pending.erase(it);
                } else {
                    ++it;
                }
            }

            // Schedule flights and update phases
            atc->scheduleFlights();
            for (auto it = active.begin(); it != active.end();) {
                Flight* f = *it;
                f->updateFuel();
                cout << "[DEBUG] Flight " << f->getFlightNumber() << " status=" << flightStatusToString(f->getStatus())
                     << " phase=" << flightPhaseToString(f->getPhase()) << " runway="
                     << (f->assignedRunway ? runwayToString(f->assignedRunway->getID()) : "None")
                     << " elapsed=" << difftime(now, f->phaseStartTime) << "\n";
                if (f->status != COMPLETED) {
                    int elapsed = difftime(now, f->phaseStartTime);
                    if ((f->phase == HOLDING || f->phase == AT_GATE) && !f->assignedRunway) {
                        cout << "[DEBUG] Flight " << f->getFlightNumber() << " waiting for runway in phase "
                             << flightPhaseToString(f->getPhase()) << "\n";
                        ++it;
                        continue;
                    }
                    if (elapsed >= f->getPhaseDuration()) {
                        cout << "[DEBUG] Progressing phase for flight " << f->getFlightNumber() << " from "
                             << flightPhaseToString(f->getPhase()) << "\n";
                        f->progressPhase();
                        if (f->status == COMPLETED && f->assignedRunway) {
                            cout << "[DEBUG] Releasing runway " << runwayToString(f->assignedRunway->getID())
                                 << " for completed flight " << f->getFlightNumber() << "\n";
                            f->assignedRunway->releaseRunway();
                            f->assignRunway(nullptr);
                        }
                    }
                    if ((f->phase == TAXI || f->phase == AT_GATE) && randomGroundFault()) {
                        cout << "\n*** GROUND FAULT: Flight " << f->getFlightNumber() << " ("
                             << f->getAirline()->name << ") towed off. ***\n";
                        if (f->assignedRunway) {
                            f->assignedRunway->releaseRunway();
                            f->assignRunway(nullptr);
                        }
                        f->setStatus(COMPLETED);
                        it = active.erase(it);
                        continue;
                    }
                }
                if (f->status == COMPLETED) {
                    cout << "[DEBUG] Removing completed flight " << f->getFlightNumber() << "\n";
                    it = active.erase(it);
                } else {
                    ++it;
                }
            }

            if (static_cast<int>(difftime(now, simulationStartTime)) % 5 == 0) {
                cout << "\n=== Simulation Update @ " << formatTime(now) << " ===\n";
                displayFlights();
                atc->displayQueues();
                displayRunwaysStatus();
                atc->displayAVNs();
                cout << "=====================================\n";
            }

            renderVisualization();
            this_thread::sleep_for(1s);
        }
        cout << "Simulation ends at: " << formatTime(time(nullptr)) << "\n";
    }
};

Flight* createFlight(vector<Airline>& airlines) {
    string flightNumber, name;
    time_t scheduledTime;
    Direction direction;
    AircraftType aircraftType;
    Airline* selectedAirline = nullptr;

    cout << "\nEnter 3-digit Flight Number (e.g., 301): ";
    while (true) {
        cin >> flightNumber;
        if (flightNumber.length() == 3 && isdigit(flightNumber[0]) && isdigit(flightNumber[1]) && isdigit(flightNumber[2])) {
            break;
        }
        cout << "Invalid input! Enter 3 digits: ";
    }

    cout << "\nSelect Airline:\n";
    for (const auto& airline : airlines) {
        cout << airline.name << "\n";
    }
    cin.ignore();
    while (true) {
        getline(cin, name);
        for (auto& airline : airlines) {
            if (name == airline.name) {
                selectedAirline = &airline;
                break;
            }
        }
        if (selectedAirline) break;
        cout << "Invalid airline! Try again: ";
    }

    if (selectedAirline->name == "PIA" || selectedAirline->name == "AirBlue") {
        aircraftType = COMMERCIAL;
    } else if (selectedAirline->name == "FedEx" || selectedAirline->name == "Blue Dart") {
        aircraftType = CARGO;
    } else {
        aircraftType = EMERGENCY;
    }

    if (aircraftType != EMERGENCY) {
        cout << "\nDirections:\n- NORTH (Int. Arrival)\n- SOUTH (Dom. Arrival)\n- EAST (Int. Departure)\n- WEST (Dom. Departure)\n";
        char choice;
        while (true) {
            cout << "Enter Direction (N/S/E/W): ";
            cin >> choice;
            choice = toupper(choice);
            if (choice == 'N') { direction = NORTH; break; }
            else if (choice == 'S') { direction = SOUTH; break; }
            else if (choice == 'E') { direction = EAST; break; }
            else if (choice == 'W') { direction = WEST; break; }
            cout << "Invalid input! Enter N/S/E/W\n";
        }
    } else {
        char choice;
        while (true) {
            cout << "Emergency Flight (A for Arrival, D for Departure): ";
            cin >> choice;
            choice = toupper(choice);
            if (choice == 'A') { direction = ARR; break; }
            else if (choice == 'D') { direction = DEP; break; }
            cout << "Invalid input! Enter A/D\n";
        }
    }

    int hour, mins;
    char colon;
    while (true) {
        cout << "Enter Scheduled Time (HH:MM): ";
        cin >> hour >> colon >> mins;
        if (colon == ':' && hour >= 0 && hour <= 23 && mins >= 0 && mins <= 59) break;
        cout << "Invalid format! Use HH:MM (24-hour)\n";
        cin.clear();
        cin.ignore(numeric_limits<streamsize>::max(), '\n');
    }

    time_t now = time(0);
    tm now_tm;
    localtime_r(&now, &now_tm);
    now_tm.tm_hour = hour;
    now_tm.tm_min = mins;
    now_tm.tm_sec = 0;
    scheduledTime = mktime(&now_tm);

    string prefix = selectedAirline->name == "PIA" ? "PK" :
                    selectedAirline->name == "AirBlue" ? "AB" :
                    selectedAirline->name == "FedEx" ? "FE" :
                    selectedAirline->name == "PAF" ? "PA" :
                    selectedAirline->name == "Blue Dart" ? "BD" : "AM";
    string fullFlightNumber = prefix + flightNumber;

    Flight* newFlight = new Flight(fullFlightNumber, selectedAirline, direction, aircraftType, scheduledTime);
    cout << "\nFlight Created:\nFlight Number: " << newFlight->getFlightNumber()
         << "\nAirline: " << newFlight->getAirline()->name
         << "\nDirection: " << getDirectionMeaning(direction)
         << "\nType: " << getAircraftTypeName(aircraftType) << "\n";
    return newFlight;
}

bool hasCargoFlight(const vector<Flight*>& flights) {
    for (auto f : flights) {
        if (f->getAircraftType() == CARGO) return true;
    }
    return false;
}

void airlinePortal(AirTrafficControl* atc) {
    cout << "\n=== Airline Portal ===\n";
    atc->displayAVNs();
    string avnID;
    double amount;
    cout << "Enter AVN ID to pay: ";
    cin >> avnID;
    cout << "Enter payment amount (PKR): ";
    cin >> amount;
    if (atc->payAVN(avnID, amount)) {
        cout << "Payment processed successfully\n";
    } else {
        cout << "Payment failed\n";
    }
}

int main() {
    srand(static_cast<unsigned int>(time(0)));
    sf::RenderWindow window(sf::VideoMode(600, 500), "AirControlX Simulation");
    Runway rwyA(RWY_A), rwyB(RWY_B), rwyC(RWY_C);
    AirTrafficControl controller(&rwyA, &rwyB, &rwyC);
    Simulation simulation(&controller, &window);

    vector<Flight*> flights;
    char again;
    cout << "Add flight details? (Y/N): ";
    cin >> again;
    cin.ignore(numeric_limits<streamsize>::max(), '\n');

    if (again == 'Y' || again == 'y') {
        do {
            cout << "\n=== Add Flight ===\n";
            Flight* f = createFlight(airlines);
            flights.push_back(f);
            cout << "\nAdd another flight? (Y/N): ";
            cin >> again;
            cin.ignore(numeric_limits<streamsize>::max(), '\n');
        } while (again == 'Y' || again == 'y');
    }

    if (!hasCargoFlight(flights)) {
        cout << "\nERROR: At least one cargo flight required.\n";
        return 1;
    }

    for (Flight* f : flights) {
        simulation.addFlight(f);
    }

    simulation.start();
    cout << "\n=== Final Schedule ===\n";
    for (auto flight : flights) {
        string runway = flight->getAssignedRunway() ? simulation.runwayToString(flight->getAssignedRunway()->getID()) : "None";
        cout << flight->getFlightNumber() << " @ " << formatTime(flight->getScheduledTime()) << " - " << runway << "\n";
    }

    airlinePortal(&controller);

    for (auto flight : flights) {
        delete flight;
    }
    return 0;
}
