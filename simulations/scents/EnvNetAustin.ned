//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

package scentssim.simulations.scents;

import scentssim.utmpc.BlendNode;
import inet.common.geometry.common.OsgGeographicCoordinateSystem;
import inet.environment.common.PhysicalEnvironment;
import inet.networklayer.configurator.nexthop.NextHopNetworkConfigurator;
import inet.physicallayer.ieee802154.packetlevel.Ieee802154NarrowbandScalarRadioMedium;
import inet.visualizer.integrated.IntegratedVisualizer;

network EnvNetAustin
{
    parameters:
        @display("bgb=520,315;bgi=background/terrain,s");
        double playgroundWidth @unit("m") = default(3000m);  // the E-W size of playground
        double playgroundHeight @unit("m") = default(3000m); // the N-S size of playground
        int numHosts = default(6);
    submodules:
        physicalEnvironment: PhysicalEnvironment {
            @display("p=100, 50");
        }
        visualizer: IntegratedVisualizer {
            parameters:
                @display("p=100, 100");
        }
        configurator: NextHopNetworkConfigurator {
            @display("p=100, 150");
        }
        radioMedium: Ieee802154NarrowbandScalarRadioMedium {
            @display("p=100, 200");
        }
        coordinateSystem: OsgGeographicCoordinateSystem {
            @display("p=100, 250");
        }
        host[numHosts]: BlendNode {
            parameters:
                mobilityType="HumanWPMobility";
            	@display("p=200, 300");
        }
}
