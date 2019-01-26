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

package scentssim.utmpc.ndlayer;

simple BlendAgent
{
    parameters:
        int advInterval @unit(ms) = default(100ms);
        int blendMode = default(2);	// FullBlend (see Constants.h) 
        int epochLength @unit(ms) = default(2000ms);
        int framework = default(0);
        int lambda @unit(ms) = default(2000ms);
        int nodeId = default(0);
        int pduLength @unit(B) = default(31B);  // length of advertising PDU
        bool stationary = default(false);

        @display("i=block/app");
        @signal[commSignal](type="long");
        @statistic[commRec](title="Comm. cost"; source="commSignal"; record=vector);
    gates:
        input lowerLayerIn @labels(Ieee802Ctrl/up);
        output lowerLayerOut @labels(Ieee802Ctrl/down);
        input upperLayerIn @loose;
        output upperLayerOut @loose;
}
