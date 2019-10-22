/*
 * BlendAgent.h
 *
 * This component runs the BLEnd-like scheduling with the given
 * parameters epochLength, advInterval and mode (from the
 * configuration ini file).  Used the random slack from the MAC base.
 *
 *      Author: liuchg1011
 */

#ifndef UTMPC_BLENDAGENT_H_
#define UTMPC_BLENDAGENT_H_

#include <omnetpp.h>

#include <map>
#include <memory>
#include <string>
#include <vector>
#include <utility>

#include "../utility/Constants.h"
#include "inet/common/INETDefs.h"
#include "inet/common/lifecycle/ILifecycle.h"
#include "inet/common/lifecycle/NodeStatus.h"

namespace scentssim {

class BlendAgent: public omnetpp::cSimpleModule, public inet::ILifecycle {
public:
    BlendAgent();
    virtual ~BlendAgent();
protected:
    // Node management
    virtual void initialize(int stage) override;
    virtual void handleMessage(omnetpp::cMessage *msg) override;
    virtual bool handleOperationStage(inet::LifecycleOperation *operation, /*int stage,*/
            inet::IDoneCallback *doneCallback) override;
    virtual int numInitStages() const override {
        return inet::NUM_INIT_STAGES;
    }
    virtual void finish() override;
    virtual bool isNodeUp();
    virtual void refreshDisplay() const override;

    // Scheduling
    virtual void cancelNextPacket();
    virtual void startTimer(NDTimer timer);
    virtual void startEpoch();

    // ND operations
    void sendBeacon();
    void startScan();
    void stopScan();
    void advComplete();
    void activateShadowBeacon(int timeToNextListenMs);

    // Neighbor management
    double updateNeighbor(int node_id, double distance = kUninitializedDouble);

    // Comm. with other layers
    void receiveMsgFromUpperLayer(const omnetpp::cMessage& msg);
    void sendMsgToUpperLayer(omnetpp::cMessage* msg);
    void receivePacketFromLowerLayer(omnetpp::cPacket* msg);

    // Data collection
    void emitRecord(NDModuleState ndState);

    // Status management
    inet::NodeStatus* node_status;
    NDModuleState ba_state;
    // Scheduling timers
    omnetpp::cMessage *warmup_timer, *adv_timer, *scan_complete_timer,
            *epoch_timer;

    // Blend params
    int node_id;
    omnetpp::cPar *par_epoch_length = nullptr;
    omnetpp::SimTime epoch_length;
    omnetpp::SimTime adv_interval;
    double scanning_period;   // L = a + b + s
    int lambda_ms;
    BlendMode blend_mode;
    int pdu_length_bytes;

    // Receive statistics
    long packetsSent = 0;
    long packetsReceived = 0;
    static omnetpp::simsignal_t comm_signal;

    int num_neighbors;
    std::vector<int> active_neighbors;

private:
    class BlendNeighbor {
    public:
        BlendNeighbor(int n_id, omnetpp::SimTime n_disc_time,
                std::string n_info) :
                node_id { n_id }, discovered_time { n_disc_time }, last_seen_time {
                        n_disc_time } {
        }
        BlendNeighbor(int n_id, omnetpp::SimTime disc_time, double dist);
        int node_id;
        double velocity;
        std::vector<std::pair<omnetpp::SimTime, double>> dist_vec;
        omnetpp::SimTime discovered_time;
        omnetpp::SimTime last_seen_time;
    };
    int beacon_num_; // Starts from 1
    int max_beacon_num_;
    long epoch_num_ = 1;
    long seq_num_ = 0;
    std::map<int, std::unique_ptr<BlendNeighbor>> neighbor_map_;
    std::string app_msg_;
    std::unique_ptr<bool[]> active_beacons_;
};

}

#endif /* UTMPC_BLENDAGENT_H_ */
