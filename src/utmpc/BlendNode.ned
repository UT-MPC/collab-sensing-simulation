//  Copyright 2018 The University of Texas at Austin
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions are
//  met:
//
//  1. Redistributions of source code must retain the above copyright
//  notice, this list of conditions and the following disclaimer.
//
//  2. Redistributions in binary form must reproduce the above copyright
//  notice, this list of conditions and the following disclaimer in the
//  documentation and/or other materials provided with the distribution.
//
//  3. Neither the name of the copyright holder nor the names of its
//  contributors may be used to endorse or promote products derived from
//  this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//  HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.



package scentssim.utmpc;

import scentssim.utmpc.applayer.SensingApplication;
import scentssim.utmpc.ndlayer.BlendAgent;
import scentssim.utmpc.linklayer_slack.Ieee802154NarrowbandInterface;
import inet.common.lifecycle.NodeStatus;
import inet.mobility.contract.IMobility;
import inet.power.contract.IEnergyStorage;

module BlendNode
{
    parameters:
        bool hasStatus = default(false);        
        int epochLengthMs @unit(ms) = default(2000ms);
        int blendNodeId = default(0);
        int lambdaMs @unit(ms) = default(2000ms);
        int numRadios = default(1);
        string energyStorageType = default("");        
        string mobilityType = default("StationaryMobility");
        bool stationaryNode = default(false);
        @display("i=device/palm");
        @networkNode();
        *.interfaceTableModule = "";
        wlan[0].typename = default("Ieee802154NarrowbandInterface");
        wlan[0].radio.antenna.mobilityModule = default("^.^.^.mobility");
        wlan[0].radio.transmitter.preambleDuration = 0s;
        wlan[0].radio.energySourceModule = default(energyStorageType != "" ? absPath(".energyStorage") : "");

    gates:
        input radioIn @directIn; // for receiving air frames

    submodules:
        app: SensingApplication {
            parameters:
                @display("b=40,24;p=173,100,col");
                epochLength = epochLengthMs;
                lambda = lambdaMs;
                nodeId = blendNodeId;
                stationary = stationaryNode;
        }
        blendagent: BlendAgent {
            parameters:
                @display("b=40,24;p=173,170,col");
                epochLength = epochLengthMs;
                lambda = lambdaMs;
                nodeId = blendNodeId;
                stationary = stationaryNode;
        }
        energyStorage: <energyStorageType> like IEnergyStorage if energyStorageType != "" {
            parameters:
                @display("p=50,100;i=block/plug;is=s");
        }
        mobility: <mobilityType> like IMobility {
            @display("p=50,152");
        }
        status: NodeStatus if hasStatus {
            @display("p=50,50");
        }
        wlan[numRadios]: Ieee802154NarrowbandInterface {
            parameters:
                @display("p=173,241;q=queue");
        }

    connections allowunconnected:
        wlan[0].radioIn <-- radioIn;
        blendagent.lowerLayerIn <-- wlan[0].upperLayerOut;
        blendagent.lowerLayerOut --> wlan[0].upperLayerIn;
        app.NDLayerIn <-- blendagent.upperLayerOut;
        app.NDLayerOut --> blendagent.upperLayerIn;
}
