#include "BlendAgent.h"

#include <algorithm>
#include <bitset>
#include <iostream>
#include <math.h>
#include <sstream>
#include <stdio.h>

#include "../applayer/SensingApplication.h"
#include "../linklayer_slack/Ieee802154Mac.h"
#include "inet/applications/ethernet/EtherApp_m.h"
#include "inet/common/ModuleAccess.h"
#include "inet/common/lifecycle/NodeOperations.h"
#include "inet/linklayer/common/Ieee802Ctrl.h"
#include "inet/networklayer/common/InterfaceTable.h"
#include "inet/common/ProtocolTag_m.h"

using scentssim::BlendAgent;
using inet::EtherAppReq;
using inet::IDoneCallback;
using inet::IEEE802CTRL_DATA;
using inet::INITSTAGE_APPLICATION_LAYER;
using inet::INITSTAGE_LOCAL;
using inet::Ieee802MessageKind;
using inet::LifecycleOperation;
using inet::NodeCrashOperation;
using inet::NodeShutdownOperation;
using inet::NodeStartOperation;
using inet::NodeStatus;
using omnetpp::SimTime;
using omnetpp::cMessage;
using omnetpp::cModule;
using omnetpp::cPacket;
using omnetpp::cRuntimeError;
using omnetpp::cSimpleModule;
using omnetpp::simsignal_t;
using std::ostringstream;
using std::string;
using std::unique_ptr;
using std::vector;
using std::pair;

Define_Module(BlendAgent);

simsignal_t BlendAgent::comm_signal = registerSignal("commSignal");

// Helper function for splitting strings
vector<string> split(const string &text, char sep) {
    vector<string> tokens;
    std::size_t start = 0, end = 0;
    while ((end = text.find(sep, start)) != string::npos) {
        if (end != start) {
            tokens.push_back(text.substr(start, end - start));
        }
        start = end + 1;
    }
    if (end != start) {
        tokens.push_back(text.substr(start));
    }
    return tokens;
}

template<typename T>
string join(const T& v, char delim) {
    ostringstream s;
    for (const auto& i : v) {
        if (&i != &v[0]) {
            s << delim;
        }
        s << i;
    }
    return s.str();
}

ulong convertVectorToUint(const vector<int>& neighbor_vec) {
    std::bitset<scentssim::kMaxNeighborCount> bitset;
    for (auto neighbor_id : neighbor_vec) {
        bitset.set(neighbor_id, true);
    }
    return bitset.to_ulong();
}

BlendAgent::BlendAgent() {
}

BlendAgent::~BlendAgent() {
    cancelAndDelete(warmup_timer);
    cancelAndDelete(adv_timer);
    cancelAndDelete(scan_complete_timer);
    cancelAndDelete(epoch_timer);

    neighbor_map_.clear();
}

void BlendAgent::initialize(int stage) {
    cSimpleModule::initialize(stage);

    if (stage == INITSTAGE_LOCAL) {
        par_epoch_length = &par("epochLength");
        epoch_length = SimTime(par_epoch_length->intValue(), SIMTIME_MS);
        adv_interval = SimTime(par("advInterval").intValue(), SIMTIME_MS);
        pdu_length_bytes = par("pduLength").intValue();
        node_id = par("nodeId").intValue();
        blend_mode = (BlendMode) par("blendMode").intValue();

        scanning_period = par("advInterval").intValue() + kTimeTransmission
                + kMaxSlackPeriod;

        // private info for debugging
        seq_num_ = 0;
        WATCH(seq_num_);

        // stats
        packetsSent = 0;
        packetsReceived = 0;
        num_neighbors = 0;
        WATCH(packetsSent);
        WATCH(packetsReceived);
        WATCH(num_neighbors);
        WATCH_VECTOR(active_neighbors);

        // initialize the timers
        warmup_timer = new cMessage("timer-warmup");
        adv_timer = new cMessage("timer-adv");
        scan_complete_timer = new cMessage("timer-scan-complete");
        epoch_timer = new cMessage("timer-epoch");

    } else if (stage == INITSTAGE_APPLICATION_LAYER) {
        node_status =
                dynamic_cast<NodeStatus *>(findContainingNode(this)->getSubmodule(
                        "status"));

        if (isNodeUp()) {
            // Initialize the active beacons according to mode
            // blend pp.5
            max_beacon_num_ = (par("epochLength").intValue()
                    / par("advInterval").intValue()) - 1;
            active_beacons_ = std::unique_ptr<bool[]>(
                    new bool[max_beacon_num_ + 1]);
            switch (blend_mode) {
            case kUBLEND:
            case kBBLEND: {
                for (int i = 1; i <= max_beacon_num_; ++i) {
                    active_beacons_[i] =
                            (i <= max_beacon_num_ / 2) ? true : false;
                }
                break;
            }
            case kFBLEND: {
                for (int i = 1; i <= max_beacon_num_; ++i) {
                    active_beacons_[i] = true;
                }
                break;
            }
            default:
                throw cRuntimeError("Unknown blend mode!");
            }

            EV_INFO << "(" << blend_mode << ") Initial beacon status:"
                           << std::boolalpha;

            for (int i = 1; i <= max_beacon_num_; ++i) {
                EV_INFO << i << ":" << active_beacons_[i] << ", ";
            }
            EV_INFO << endl;

            ba_state = kNDStateWarmUp;
            startTimer(kTimerWarmUp);

            emitRecord(ba_state);

            if (hasGUI())
                refreshDisplay();
        }
    }

}

void BlendAgent::handleMessage(cMessage *msg) {
    EV_DEBUG << "Received msg(" << msg->getDisplayString() << "): "
                    << msg->getName() << " at " << simTime() << "\n";

    if (msg->isSelfMessage()) {
        if (msg == warmup_timer) {
            startEpoch();
        } else if (msg == adv_timer) {
            if (active_beacons_[beacon_num_]) {
                sendBeacon(); //adv_timer will be set after transmission
            } else {
                ++beacon_num_; // Skip inactive beacons
                startTimer(kTimerAdv);
            }
        } else if (msg == scan_complete_timer) {
            stopScan();
            cMessage* new_epoch = new cMessage("new epoch");
            new_epoch->setKind(kKindNewEpochBeacons);
            sendMsgToUpperLayer(new_epoch);
            sendBeacon();
        } else if (msg == epoch_timer) {
            startEpoch();
        }
    } else if (msg->arrivedOn("lowerLayerIn")) {
        if (msg->getKind() == TRANSMISSION_COMPLETE) {
            advComplete();
            delete msg;
        } else {
            if (ba_state == kNDStateScan) {
                receivePacketFromLowerLayer(check_and_cast<cPacket *>(msg));
                // upper layer needs to delete this message after processing
            } else {
                EV_INFO << "[BA] Receiver is not active. Dropping packet..."
                               << endl;
                delete msg;
            }
        }
    } else if (msg->arrivedOn("upperLayerIn")) {
        receiveMsgFromUpperLayer(*msg);
        delete msg;
    }
}

void BlendAgent::startTimer(NDTimer timer) {
    switch (timer) {
    case kTimerWarmUp: {
        // Warmup period
        SimTime warmup_period = SimTime(
                intuniform(0, par_epoch_length->intValue()), SIMTIME_MS);
        EV_DEBUG << "[BA] Node warm up period set to " << warmup_period << "s."
                        << endl;
        scheduleAt(warmup_period, warmup_timer);
        break;
    }
    case kTimerAdv: {
        SimTime adv_time = simTime() + adv_interval;
        EV_DEBUG << "[BA] Next adv timer set to " << adv_time
                        << "s, beacon number:" << beacon_num_ << endl;
        scheduleAt(adv_time, adv_timer);
        break;
    }
    case kTimerScanComplete: {
        SimTime scan_complete_time = simTime()
                + SimTime(scanning_period, SIMTIME_MS);
        EV_DEBUG << "[BA] Scan complete timer set to " << scan_complete_time
                        << "s." << endl;
        if (scan_complete_timer->isScheduled()) {
            cancelEvent(scan_complete_timer);
        }
        scheduleAt(scan_complete_time, scan_complete_timer);
        break;
    }
    case kTimerEpochStart: {
        scheduleAt(simTime() + epoch_length, epoch_timer);
        break;
    }
    default: {
        throw cRuntimeError("Unknown timer type!");
    }
    }
}

void BlendAgent::startEpoch() {
    // Cancel timers
    if (warmup_timer->isScheduled()) {
        cancelEvent(warmup_timer);
    } else if (adv_timer->isScheduled()) {
        cancelEvent(adv_timer);
    } else if (scan_complete_timer->isScheduled()) {
        cancelEvent(scan_complete_timer);
    } else if (epoch_timer->isScheduled()) {
        cancelEvent(epoch_timer);
    }

    beacon_num_ = 1;

    startTimer(kTimerEpochStart);

    if (ba_state == kNDStateWarmUp) {
        EV_INFO << "[BA] Node " << node_id
                       << " finished warming up. Now start the first epoch: "
                       << simTime() << "s." << endl;
    } else {
        EV_INFO << "[BA] Node " << node_id << ": new epoch " << epoch_num_
                       << " starts at " << simTime()
                       << "s. Next epoch timer scheduled." << endl;
    }

    ba_state = kNDStateIdle;
    emitRecord(ba_state);

    ++epoch_num_;
    sendBeacon();
}

bool BlendAgent::handleOperationStage(LifecycleOperation* operation, int stage,
        IDoneCallback* doneCallback) {
    Enter_Method_Silent();
    if (dynamic_cast<NodeStartOperation *>(operation)) {
        EV_INFO << "stage=STAGE_APPLICATION_LAYER, should start schedule packet now.\n";
    }
    else if (dynamic_cast<NodeShutdownOperation *>(operation)) {
        if ((NodeShutdownOperation::Stage)stage == NodeShutdownOperation::STAGE_APPLICATION_LAYER)
        cancelNextPacket();
    }
    else if (dynamic_cast<NodeCrashOperation *>(operation)) {
        if ((NodeCrashOperation::Stage)stage == NodeCrashOperation::STAGE_CRASH)
        cancelNextPacket();
    }
    else
    throw cRuntimeError("Unsupported lifecycle operation '%s'", operation->getClassName());
    return true;
}

bool BlendAgent::isNodeUp() {
    return !node_status || node_status->getState() == NodeStatus::UP;
}

void BlendAgent::refreshDisplay() const {
    char display_str[20];
    sprintf(display_str, "#Neighbors:%d", num_neighbors);
    getParentModule()->getDisplayString().setTagArg("t", 0, display_str);
}

void BlendAgent::cancelNextPacket() {
}

void BlendAgent::receivePacketFromLowerLayer(cPacket* msg) {
    NDModuleState old_state = ba_state;
    ba_state = kNDStateRecv;
    emitRecord(ba_state);
    std::string msg_txt(msg->getName());
    EV_DEBUG << "[BA] Received msg from lower layer '" << msg_txt << endl;

    string::size_type dlt_idx = msg_txt.find(kBcnDelimiter);
    if (dlt_idx == string::npos) {
        throw cRuntimeError("Message parsing error!");
    }
    string bcn_info = msg_txt.substr(0, dlt_idx);
    string rcvd_app_msg = msg_txt.substr(dlt_idx + 1);

    // process header
    vector<string> tokens = split(bcn_info, kBcnInfoDelimiter);
    assert(tokens.size() > 0);
    int neighbor_id = std::stoi(tokens[0]);

    // Activate shadow beacons when in BBlend mode
    if (blend_mode == kBBLEND) {
        int time_to_next_listen = std::stoi(tokens[2]);
        activateShadowBeacon(time_to_next_listen);
    }

    ++packetsReceived;

    Framework fw = (Framework) par("framework").intValue();
//    EV_DEBUG << "receivePacketFromLowerLayer(), framework=" << fw << endl;
    switch (fw) {
    case kFrameworkNone: {
        msg->setName(rcvd_app_msg.c_str());
        updateNeighbor(neighbor_id);
        msg->addPar("neighbor_set");
        msg->par("neighbor_set").setLongValue(
                convertVectorToUint(active_neighbors));
        break;
    }
    case kFrameworkScents: {
        // send the app msg to app layer(with distance par.)
        char mod_str[20];
        sprintf(mod_str, "host[%d]", neighbor_id);
        cModule* neighbor_host = omnetpp::getSimulation()->getModuleByPath(mod_str);
        if (neighbor_host == nullptr) {
            sprintf(mod_str, "stationary[%d]", neighbor_id - 1);
            neighbor_host = omnetpp::getSimulation()->getModuleByPath(mod_str);
        }
        assert(neighbor_host != nullptr);

        msg->addPar("neighbor_id");
        msg->par("neighbor_id").setLongValue(neighbor_id);

        cModule* host = inet::getContainingNode(this);
        IMobility *imob = check_and_cast<IMobility*>(
                host->getSubmodule("mobility"));
        const Coord& coord1 = imob->getCurrentPosition();
        imob = check_and_cast<IMobility*>(neighbor_host->getSubmodule("mobility"));
        const Coord& coord2 = imob->getCurrentPosition();
        double dist_m = sqrt(
                (coord1.x - coord2.x) * (coord1.x - coord2.x)
                        + (coord1.y - coord2.y) * (coord1.y - coord2.y));
        msg->addPar("dist");
        msg->par("dist").setDoubleValue(dist_m);
        msg->setName(rcvd_app_msg.c_str()); // pass only the app related info up

        double l = updateNeighbor(neighbor_id, dist_m);
        if (l != kUninitializedDouble) {
            msg->addPar("stability");
            msg->par("stability").setDoubleValue(l);
        }
        //TODO(liuchg): Remove this part after adding cap. vector to the beacon.
        SensingApplication *app = check_and_cast<SensingApplication*>(
                neighbor_host->getSubmodule("app"));
        std::vector<ContextType> cap = app->sensing_capabilities;
        string cap_str = join(cap, kDelimiter);
        msg->addPar("cap");
        msg->par("cap").setStringValue(cap_str.c_str());
        break;
    }
    default: {
        throw cRuntimeError(
                "[BlendAgent::receivePacketFromLowerLayer]Unknown framework");
    }
    }

    ba_state = old_state;
    emitRecord(ba_state);

    sendMsgToUpperLayer(msg);
}

void BlendAgent::sendBeacon() {
    EV_DEBUG << "[BA] Sending beacon:#" << beacon_num_ << ",epoch#"
                    << epoch_num_ << ")" << " with content:" << app_msg_ << endl;

    double ttne_ms = kUninitializedInt;
    if (epoch_timer->isScheduled()) {
        ttne_ms = (epoch_timer->getArrivalTime() - simTime()).dbl() * 1000;
        if (blend_mode != kUBLEND) {
            ttne_ms += kTimeTransmission;
        }
    }
    char bcn_raw[kUncompressedLimit];
    // Compressing...
    sprintf(bcn_raw, "%d-%d-%d|%s", node_id, beacon_num_++,
            (int) std::ceil(ttne_ms), app_msg_.c_str());

    Packet *adv_packet = new Packet(bcn_raw, IEEE802CTRL_DATA);
    const auto& data = makeShared<EtherAppReq>();
    data->setRequestId(seq_num_);
    data->setChunkLength(B(pdu_length_bytes));
    adv_packet->insertAtBack(data);
    send(adv_packet, "lowerLayerOut");
    ba_state = kNDStateAdv;
    emitRecord(ba_state);
    ++packetsSent;
}

void BlendAgent::startScan() {
    EV_DEBUG << "[BA] startScan(): ba_state=" << ba_state << " at " << simTime() << endl;
    ba_state = kNDStateScan;
    emitRecord(ba_state);
    startTimer(kTimerScanComplete);
}
void BlendAgent::stopScan() {
    ba_state = kNDStateIdle;
    emitRecord(ba_state);
}

void BlendAgent::advComplete() {
    EV_DEBUG << "From lower layer: TRANSMISSION_COMPLETE. Going into 'idle'."
                    << simTime() << endl;
    ba_state = kNDStateIdle;
    emitRecord(ba_state);
    if (beacon_num_ == 2) {
        startScan();
    } else {
        startTimer(kTimerAdv);
    }
}

void BlendAgent::activateShadowBeacon(int timeToNextListenMs) {
    assert(timeToNextListenMs > 0);
    assert(scan_complete_timer->isScheduled());
    // Get time from scan complete to the target beacon time
    int scan_complete_ms =
            (scan_complete_timer->getArrivalTime() - simTime()).dbl() * 1000;
    if (scan_complete_ms >= timeToNextListenMs) {
        return;
    } else {
        for (int i = 2; i <= max_beacon_num_; ++i) {
            if ((i - 1) * (adv_interval.dbl() * 1000)
                    >= timeToNextListenMs - scan_complete_ms) {
                if (!active_beacons_[i]) {
                    active_beacons_[i] = true;
                    EV_INFO << "[ND] Turning on shadow beacon " << i
                                   << " in BBlend mode.\n";
                }
                break;
            }
        }
        EV_INFO << "(" << blend_mode << ") Updated beacon status:"
                       << std::boolalpha;
        for (int i = 1; i <= max_beacon_num_; ++i) {
            EV_INFO << i << ":" << active_beacons_[i] << ", ";
        }
        EV_INFO << endl;
    }

}

void BlendAgent::finish() {
}

double BlendAgent::updateNeighbor(int node_id, double distance) {
    double l = kUninitializedDouble;
    if (distance > kCommRangeM)
        return 0;
    if (neighbor_map_.find(node_id) == neighbor_map_.end()) {
        neighbor_map_[node_id] = unique_ptr < BlendNeighbor
                > (new BlendNeighbor(node_id, simTime(), distance));
        active_neighbors.push_back(node_id);
        EV_DEBUG << "[BA] New neighbor found: " << node_id << endl;
    } else {
        neighbor_map_[node_id]->last_seen_time = simTime();
//        EV_DEBUG << "[BA] Update last seen time of node " << node_id << " to "
//                        << neighbor_map_[node_id]->last_seen_time << "s."
//                        << endl;
        pair<SimTime, double> p(simTime(), distance);
        neighbor_map_[node_id]->dist_vec.insert(
                neighbor_map_[node_id]->dist_vec.begin(), p);
    }
    // remove disconnected neighbors
    for (auto it = neighbor_map_.begin(); it != neighbor_map_.end();) {
        if (it->second->last_seen_time
                < (simTime() - kNumEpochBeforeLost * epoch_length)) {
            active_neighbors.erase(
                    std::remove(active_neighbors.begin(),
                            active_neighbors.end(), it->second->node_id),
                    active_neighbors.end());
            it = neighbor_map_.erase(it);
        } else {
            ++it;
        }
    }
    if ((Framework) par("framework").intValue() == kFrameworkScents) {
        // Compute the stability.
        auto& neighbor = neighbor_map_[node_id];
        // Remove the out-dated distance records
        for (auto it = neighbor->dist_vec.begin();
                it != neighbor->dist_vec.end();) {
            if (it->first < (simTime() - kNumEpochBeforeLost * epoch_length)) {
                it = neighbor->dist_vec.erase(it);
            } else {
                ++it;
            }
        }
        if (neighbor->dist_vec.size() >= 2) {
            double delta_time = (neighbor->dist_vec.front().first
                    - neighbor->dist_vec.back().first).dbl();
            double delta_dist = neighbor->dist_vec.front().second
                    - neighbor->dist_vec.back().second;
            double v = delta_dist / delta_time;

            lambda_ms = par("lambda");
            double r_thrd = kCommRangeM - (lambda_ms / 1000) * v;

            l = 1 / (1 + std::exp(0.1 * (distance - (r_thrd / 2)))); // https://www.desmos.com/calculator/agxuc5gip8

            EV_DEBUG << "[BA]Update neighbor velocity:" << this->node_id << "-"
                            << node_id << ": " << v << "m/s, d=" << distance
                            << "m, l=" << l << endl;
//                        << " (from " << neighbor->dist_vec.size()
//                        << " records, thrd=" << r_thrd << ")\n";
        }
    }

    int prev_num = num_neighbors;
    num_neighbors = neighbor_map_.size();
    if (prev_num != num_neighbors && hasGUI()) {
        refreshDisplay();
    }
    return l;
}

// App layer is responsible for deallocations.
void BlendAgent::sendMsgToUpperLayer(cMessage* msg) {
    send(msg, "upperLayerOut");
}

void BlendAgent::receiveMsgFromUpperLayer(const cMessage& msg) {
    string msg_str(msg.getName());
    EV_DEBUG << "[BA] Received msg from app layer: " << msg_str << endl;
    app_msg_ = msg_str;
}

void BlendAgent::emitRecord(NDModuleState ndState) {
    if (par("stationary").boolValue() == true)
        return;
//    EV_DEBUG << "[BA] emitRecord: " << ndState << endl;
    emit(comm_signal, node_id);
    emit(comm_signal, ndState);
}

// Inner class BlendNeighbor
BlendAgent::BlendNeighbor::BlendNeighbor(int n_id, SimTime disc_time,
        double dist) {
    node_id = n_id;
    pair<SimTime, double> p(disc_time, dist);
    dist_vec.push_back(p);
    discovered_time = disc_time;
    last_seen_time = disc_time;
}
