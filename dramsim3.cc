/*
 * Copyright (c) 2013 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "mem/dramsim3.hh"

#include "base/callback.hh"
#include "base/trace.hh"
#include "debug/DRAMsim3.hh"
#include "debug/Drain.hh"
#include "sim/system.hh"

#include <cstdint>
#include <random>

namespace gem5
{

namespace memory
{

DRAMsim3::DRAMsim3(const Params &p) :
    AbstractMemory(p),
    port(name() + ".port", *this),
    read_cb(std::bind(&DRAMsim3::readComplete,
                      this, 0, std::placeholders::_1,
                      std::placeholders::_2)),
    write_cb(std::bind(&DRAMsim3::writeComplete,
                       this, 0, std::placeholders::_1,
                       std::placeholders::_2)),
    refresh_cb(std::bind(&DRAMsim3::refreshComplete,
                       this, 0, std::placeholders::_1,
                       std::placeholders::_2, std::placeholders::_3)),
    wrapper(p.configFile, p.filePath, read_cb, write_cb, refresh_cb),
    retryReq(false), retryResp(false), startTick(0),
    nbrOutstandingReads(0), nbrOutstandingWrites(0),
    sendResponseEvent([this]{ sendResponse(); }, name()),
    tickEvent([this]{ tick(); }, name())
{
    DPRINTF(DRAMsim3,
            "Instantiated DRAMsim3 with clock %d ns and queue size %d\n",
            wrapper.clockPeriod(), wrapper.queueSize());

    // Get dram mapping
    config = wrapper.GetConfig();

    para_rng = std::minstd_rand(0);

    // Register a callback to compensate for the destructor not
    // being called. The callback prints the DRAMsim3 stats.
    registerExitCallback([this]() { wrapper.printStats(); });
}

void
DRAMsim3::init()
{
    AbstractMemory::init();

    if (!port.isConnected()) {
        fatal("DRAMsim3 %s is unconnected!\n", name());
    } else {
        port.sendRangeChange();
    }

    if (system()->cacheLineSize() != wrapper.burstSize())
        fatal("DRAMsim3 burst size %d does not match cache line size %d\n",
              wrapper.burstSize(), system()->cacheLineSize());
}

void
DRAMsim3::startup()
{
    startTick = curTick();

    // kick off the clock ticks
    schedule(tickEvent, clockEdge());
}

void
DRAMsim3::resetStats() {
    wrapper.resetStats();
}

void
DRAMsim3::sendResponse()
{
    assert(!retryResp);
    assert(!responseQueue.empty());

    DPRINTF(DRAMsim3, "Attempting to send response\n");

    bool success = port.sendTimingResp(responseQueue.front());
    if (success) {
        responseQueue.pop_front();

        DPRINTF(DRAMsim3, "Have %d read, %d write, %d responses outstanding\n",
                nbrOutstandingReads, nbrOutstandingWrites,
                responseQueue.size());

        if (!responseQueue.empty() && !sendResponseEvent.scheduled())
            schedule(sendResponseEvent, curTick());

        if (nbrOutstanding() == 0)
            signalDrainDone();
    } else {
        retryResp = true;

        DPRINTF(DRAMsim3, "Waiting for response retry\n");

        assert(!sendResponseEvent.scheduled());
    }
}

unsigned int
DRAMsim3::nbrOutstanding() const
{
    return nbrOutstandingReads + nbrOutstandingWrites + responseQueue.size();
}

void
DRAMsim3::tick()
{
    // Only tick when it's timing mode
    if (system()->isTimingMode()) {
        wrapper.tick();

        // is the connected port waiting for a retry, if so check the
        // state and send a retry if conditions have changed
        if (retryReq && nbrOutstanding() < wrapper.queueSize()) {
            retryReq = false;
            port.sendRetryReq();
        }
    } else {
        // erase both hammer and flipped state since we run in a non-timing
        // CPU that does not execute the callbacks
        // therefore we don't know if the memory was written in between and
        // flips should therefore be possible again
        hammer_count.clear();
        flipped.clear();
    }

    schedule(tickEvent,
        curTick() + wrapper.clockPeriod() * sim_clock::as_int::ns);
}

Tick
DRAMsim3::recvAtomic(PacketPtr pkt)
{
    access(pkt);

    // 50 ns is just an arbitrary value at this point
    return pkt->cacheResponding() ? 0 : 50000;
}

void
DRAMsim3::recvFunctional(PacketPtr pkt)
{
    pkt->pushLabel(name());

    functionalAccess(pkt);

    // potentially update the packets in our response queue as well
    for (auto i = responseQueue.begin(); i != responseQueue.end(); ++i)
        pkt->trySatisfyFunctional(*i);

    pkt->popLabel();
}

bool
DRAMsim3::recvTimingReq(PacketPtr pkt)
{
    // if a cache is responding, sink the packet without further action
    if (pkt->cacheResponding()) {
        pendingDelete.reset(pkt);
        return true;
    }

    // we should not get a new request after committing to retry the
    // current one, but unfortunately the CPU violates this rule, so
    // simply ignore it for now
    if (retryReq)
        return false;

    // if we cannot accept we need to send a retry once progress can
    // be made
    bool can_accept = nbrOutstanding() < wrapper.queueSize();

    // keep track of the transaction
    if (pkt->isRead()) {
        if (can_accept) {
            outstandingReads[pkt->getAddr()].push(pkt);

            // we count a transaction as outstanding until it has left the
            // queue in the controller, and the response has been sent
            // back, note that this will differ for reads and writes
            ++nbrOutstandingReads;
        }
    } else if (pkt->isWrite()) {
        if (can_accept) {
            outstandingWrites[pkt->getAddr()].push(pkt);

            ++nbrOutstandingWrites;

            // perform the access for writes
            accessAndRespond(pkt);
        }
    } else {
        // keep it simple and just respond if necessary
        accessAndRespond(pkt);
        return true;
    }

    if (can_accept) {
        // we should never have a situation when we think there is space,
        // and there isn't
        assert(wrapper.canAccept(pkt->getAddr(), pkt->isWrite()));

        DPRINTF(DRAMsim3, "Enqueueing address %lld\n", pkt->getAddr());

        // @todo what about the granularity here, implicit assumption that
        // a transaction matches the burst size of the memory (which we
        // cannot determine without parsing the ini file ourselves)
        wrapper.enqueue(pkt->getAddr(), pkt->isWrite());

        return true;
    } else {
        retryReq = true;
        return false;
    }
}

void
DRAMsim3::recvRespRetry()
{
    DPRINTF(DRAMsim3, "Retrying\n");

    assert(retryResp);
    retryResp = false;
    sendResponse();
}

void
DRAMsim3::accessAndRespond(PacketPtr pkt)
{
    DPRINTF(DRAMsim3, "Access for address %lld\n", pkt->getAddr());

    bool needsResponse = pkt->needsResponse();

    // do the actual memory access which also turns the packet into a
    // response
    access(pkt);

    // turn packet around to go back to requestor if response expected
    if (needsResponse) {
        // access already turned the packet into a response
        assert(pkt->isResponse());
        // Here we pay for xbar additional delay and to process the payload
        // of the packet.
        Tick time = curTick() + pkt->headerDelay + pkt->payloadDelay;
        // Reset the timings of the packet
        pkt->headerDelay = pkt->payloadDelay = 0;

        DPRINTF(DRAMsim3, "Queuing response for address %lld\n",
                pkt->getAddr());

        // queue it to be sent back
        responseQueue.push_back(pkt);

        // if we are not already waiting for a retry, or are scheduled
        // to send a response, schedule an event
        if (!retryResp && !sendResponseEvent.scheduled())
            schedule(sendResponseEvent, time);
    } else {
        // queue the packet for deletion
        pendingDelete.reset(pkt);
    }
}

unsigned long long counts[10] = {0};

inline double DRAMsim3::gen_proba(uint64_t addr) {
    auto pp = probabilities.find(addr);
    if (pp == probabilities.end()) {
        std::ranlux48_base gen(addr);
        probabilities[addr] = std::generate_canonical<double, 10>(gen);
        return probabilities[addr];
    } else {
        return pp->second;
    }
}

void DRAMsim3::PARA(int channel, int rank, int bankgroup, int bank, int row) {
    for (int dist=-5; dist<=5; dist++) {
        if (dist == 0 || row+dist < 0 || row+dist>=config->rows) {
            continue;
        }
        double rand = std::generate_canonical<double, 10>(para_rng);
        if (rand > config->para_proba) {
            continue;
        }
        uint64_t target = config->ReverseAddressMapping(channel, rank, bankgroup, bank, row+dist, 0);
        hammer_count.erase(target);
    }
}

void DRAMsim3::TRR(int channel, int rank, int bankgroup, int bank, int row) {
    for (int dist=-5; dist<=5; dist++) {
        if (dist == 0 || row+dist < 0 || row+dist>=config->rows) {
            continue;
        }
        uint64_t target = config->ReverseAddressMapping(channel, rank, bankgroup, bank, row+dist, 0);
        if(trr_count.find(target) == trr_count.end()) {
            trr_count[target] = 1;
        } else {
            trr_count[target]++;
        }

        if (trr_count[target] > config->trr_threshold) {
            hammer_count.erase(target);
        }
    }
}

void DRAMsim3::readComplete(unsigned id, uint64_t addr, bool bufferhit)
{
    DPRINTF(DRAMsim3, "Read to address %lld complete\n", addr);

    auto a = config->AddressMapping(addr);
    int channel = a.channel;
    int rank = a.rank;
    int bankgroup = a.bankgroup;
    int bank = a.bank;
    int row = a.row;

    // get the outstanding reads for the address in question
    auto p = outstandingReads.find(addr);
    assert(p != outstandingReads.end());

    // first in first out, which is not necessarily true, but it is
    // the best we can do at this point
    PacketPtr pkt = p->second.front();
    p->second.pop();

    if (p->second.empty())
        outstandingReads.erase(p);

    // no need to check for drain here as the next call will add a
    // response to the response queue straight away
    assert(nbrOutstandingReads != 0);
    --nbrOutstandingReads;

    // perform the actual memory access
    accessAndRespond(pkt);

    counts[channel]++;

    // Memory corruption code
    hammer_count.erase(addr);

    // no rowhammer effects when rowbuffer hit
    if (bufferhit) {
        return;
    }

    for (int dist=-5; dist<=5; dist++) {
        if (dist == 0 || row+dist < 0 || row+dist>=config->rows) {
            continue;
        }
        double add = 0;
        switch (abs(dist)) {
            case 5:
                add = config->inc_dist_5;
                break;
            case 4:
                add = config->inc_dist_4;
                break;
            case 3:
                add = config->inc_dist_3;
                break;
            case 2:
                add = config->inc_dist_2;
                break;
            case 1:
                add = config->inc_dist_1;
                break;
        }

        if (add == 0) {
            // We don't increment for this distance
            continue;
        }

        uint64_t flipped_row_base = config->ReverseAddressMapping(channel, rank, bankgroup, bank, row+dist, 0);
        if(hammer_count.find(flipped_row_base) == hammer_count.end()) {
            hammer_count.insert(std::make_pair(flipped_row_base, add));
            // Don't check against threshold here since we should be way below
            continue;
        }

        hammer_count[flipped_row_base] += add;
        if (hammer_count[flipped_row_base] < config->hc_first) {
            continue;
        }

        double row_flip_rate = config->hc_last_bitflip_rate
                *std::min((hammer_count[flipped_row_base]-config->hc_first)/(config->hc_last-config->hc_first), 1.0)
                *64; // * bits in quadword, since we decide for one quadword here
        for (uint64_t quad=flipped_row_base; quad<flipped_row_base+config->row_size; quad+=sizeof(uint64_t)) {
            if (flipped.find(quad) != flipped.end()) {
                // already flipped
                continue;
            }

            // probabilisticly flip quadword
            if (gen_proba(quad) > row_flip_rate) {
                // no flip
                continue;
            }

            flipped.insert(quad);

            // this xor makes sure that this is not the same probability as for gen_proba
            uint64_t mask = 0;
            if (config->flip_mask) {
                mask = config->flip_mask;
            } else {
                std::mt19937 gen(quad ^ 0xcafecafecafecafe);
                double flipped_bits_ran = std::generate_canonical<double, 10>(gen);
                int flipped_bits;
                if (flipped_bits_ran <= config->proba_1_bit_flipped) {
                    flipped_bits = 1;
                } else if (flipped_bits_ran <= config->proba_1_bit_flipped
                                              +config->proba_2_bit_flipped) {
                    flipped_bits = 2;
                } else if (flipped_bits_ran <= config->proba_1_bit_flipped
                                              +config->proba_2_bit_flipped
                                              +config->proba_3_bit_flipped) {
                    flipped_bits = 3;
                } else {
                    flipped_bits = 4;
                }

                std::uniform_int_distribution<> distrib(0, 63);
                for (int j = 0; j<flipped_bits; j++) {
                    int pos;
                    // find position that is not yet taken
                    do {
                        pos = distrib(gen);
                    } while (mask & (((uint64_t)1) << pos));
                    mask |= ((uint64_t)1) << pos;
                }
            }

            // flip with mask
            *(uint64_t*)this->toHostAddr(quad) ^= mask;
        }
    }

    if (config->para_enabled) {
        PARA(channel, rank, bankgroup, bank, row);
    }
    if (config->trr_enabled) {
        TRR(channel, rank, bankgroup, bank, row);
    }
}

void DRAMsim3::writeComplete(unsigned id, uint64_t addr, bool bufferhit)
{
    DPRINTF(DRAMsim3, "Write to address %lld complete\n", addr);

    // get the outstanding reads for the address in question
    auto p = outstandingWrites.find(addr);
    assert(p != outstandingWrites.end());

    // we have already responded, and this is only to keep track of
    // what is outstanding
    p->second.pop();
    if (p->second.empty())
        outstandingWrites.erase(p);

    assert(nbrOutstandingWrites != 0);
    --nbrOutstandingWrites;

    if (nbrOutstanding() == 0)
        signalDrainDone();

    // Memory corruption code
    hammer_count.erase(addr);
    for (uint64_t quad=addr; quad<addr+config->row_size; quad+=sizeof(uint64_t)) {
        flipped.erase(quad);
    }
}

void DRAMsim3::refreshComplete(unsigned id, int channel, int bankgroup, int bank)
{
    int maxcount = 0;;
    for(auto it= hammer_count.begin(); it != hammer_count.end();){
        dramsim3::Address a = config->AddressMapping(it->first);
        if((a.channel == channel) &&
           ((a.bankgroup == bankgroup) || (bankgroup == -1)) &&
           ((a.bank == bank) || (bank == -1))){
                maxcount = it->second > maxcount ? it->second : maxcount;
                // NOTE: flipped is not erased here since we dont want to flip mutliple times
                it = hammer_count.erase(it);
        } else {
            it++;
        }
    }
    DPRINTF(DRAMsim3, "Refresh at channel %d bankgroup %d bank %d complete\n", channel, bankgroup, bank);
    counts[channel] = 0;
}

Port&
DRAMsim3::getPort(const std::string &if_name, PortID idx)
{
    if (if_name != "port") {
        return ClockedObject::getPort(if_name, idx);
    } else {
        return port;
    }
}

DrainState
DRAMsim3::drain()
{
    // check our outstanding reads and writes and if any they need to
    // drain
    return nbrOutstanding() != 0 ? DrainState::Draining : DrainState::Drained;
}

DRAMsim3::MemoryPort::MemoryPort(const std::string& _name,
                                 DRAMsim3& _memory)
    : ResponsePort(_name, &_memory), mem(_memory)
{ }

AddrRangeList
DRAMsim3::MemoryPort::getAddrRanges() const
{
    AddrRangeList ranges;
    ranges.push_back(mem.getAddrRange());
    return ranges;
}

Tick
DRAMsim3::MemoryPort::recvAtomic(PacketPtr pkt)
{
    return mem.recvAtomic(pkt);
}

void
DRAMsim3::MemoryPort::recvFunctional(PacketPtr pkt)
{
    mem.recvFunctional(pkt);
}

bool
DRAMsim3::MemoryPort::recvTimingReq(PacketPtr pkt)
{
    // pass it to the memory controller
    return mem.recvTimingReq(pkt);
}

void
DRAMsim3::MemoryPort::recvRespRetry()
{
    mem.recvRespRetry();
}

} // namespace memory
} // namespace gem5
