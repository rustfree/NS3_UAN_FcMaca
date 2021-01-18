/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2009 University of Washington
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Leonard Tracy <lentracy@gmail.com>
 */

#include "uan-header-fc.h"
#include "ns3/mac8-address.h"

static const uint16_t ARP_PROT_NUMBER = 0x0806;
static const uint16_t IPV4_PROT_NUMBER = 0x0800;
static const uint16_t IPV6_PROT_NUMBER = 0x86DD;

namespace ns3 {

NS_OBJECT_ENSURE_REGISTERED (UanHeaderFc);



UanHeaderFc::UanHeaderFc ()
{
}

UanHeaderFc::UanHeaderFc (const Mac8Address src, const Mac8Address dest, uint8_t type, uint8_t protocolNumber)
  : Header (),
  m_dest (dest),
  m_src (src)
{
  SetProtocolNumber (protocolNumber);
  m_uanProtocolBits.m_type = type;
  SetUid(0);
}

TypeId
UanHeaderFc::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::UanHeaderFc")
    .SetParent<Header> ()
    .SetGroupName ("Uan")
    .AddConstructor<UanHeaderFc> ()
  ;
  return tid;
}

TypeId
UanHeaderFc::GetInstanceTypeId (void) const
{
  return GetTypeId ();
}
UanHeaderFc::~UanHeaderFc ()
{
}


void
UanHeaderFc::SetDest (Mac8Address dest)
{
  m_dest = dest;
}
void
UanHeaderFc::SetSrc (Mac8Address src)
{
  m_src = src;
}

void
UanHeaderFc::SetType (uint8_t type)
{
  m_uanProtocolBits.m_type = type;
}

void
UanHeaderFc::SetProtocolNumber (uint16_t protocolNumber)
{
  if (protocolNumber == 0)
    m_uanProtocolBits.m_protocolNumber = 0;
  else if (protocolNumber == IPV4_PROT_NUMBER)
    m_uanProtocolBits.m_protocolNumber = 1;
  else if (protocolNumber == ARP_PROT_NUMBER)
    m_uanProtocolBits.m_protocolNumber = 2;
  else if (protocolNumber == IPV6_PROT_NUMBER)
    m_uanProtocolBits.m_protocolNumber = 3;
  else
    NS_ASSERT_MSG (false, "UanHeaderFc::SetProtocolNumber(): Protocol not supported");
}

void
UanHeaderFc::SetUid (uint16_t uid)
{
    m_uid = uid;
}

void 
UanHeaderFc::SetQueNum(uint16_t queNum){
    m_queNum = queNum;
}

Mac8Address
UanHeaderFc::GetDest (void) const
{
  return m_dest;
}
Mac8Address
UanHeaderFc::GetSrc (void) const
{
  return m_src;
}
uint8_t
UanHeaderFc::GetType (void) const
{
  return m_uanProtocolBits.m_type;
}

uint16_t
UanHeaderFc::GetProtocolNumber (void) const
{
  if (m_uanProtocolBits.m_protocolNumber == 1)
    return IPV4_PROT_NUMBER;
  if (m_uanProtocolBits.m_protocolNumber == 2)
    return ARP_PROT_NUMBER;
  if (m_uanProtocolBits.m_protocolNumber == 3)
    return IPV6_PROT_NUMBER;
  return 0;
}

uint16_t 
UanHeaderFc::GetUid(void) const{
    return m_uid;
}

uint16_t 
UanHeaderFc::GetQueNum(void) const{
    return m_queNum;
}



// Inherrited methods

uint32_t
UanHeaderFc::GetSerializedSize (void) const
{
  return 1 + 1 + 1 + 2 + 2;
}

void
UanHeaderFc::Serialize (Buffer::Iterator start) const
{
  uint8_t address = 0;
  m_src.CopyTo (&address);
  start.WriteU8 (address);
  m_dest.CopyTo (&address);
  start.WriteU8 (address);
  char tmp = m_uanProtocolBits.m_type;
  tmp = tmp << 4;
  tmp += m_uanProtocolBits.m_protocolNumber;
  start.WriteU8 (tmp);
  start.WriteU16(m_uid);
  start.WriteU16(m_queNum);
}

uint32_t
UanHeaderFc::Deserialize (Buffer::Iterator start)
{
  Buffer::Iterator rbuf = start;

  m_src = Mac8Address (rbuf.ReadU8 ());
  m_dest = Mac8Address (rbuf.ReadU8 ());
  char tmp = rbuf.ReadU8 ();
  m_uanProtocolBits.m_type = tmp >> 4;
  m_uanProtocolBits.m_protocolNumber = tmp;
  m_uid = rbuf.ReadU16();
  m_queNum = rbuf.ReadU16();

  return rbuf.GetDistanceFrom (start);
}

void
UanHeaderFc::Print (std::ostream &os) const
{
  os << "UAN src=" << m_src << " dest=" << m_dest << " type=" << (uint32_t) m_uanProtocolBits.m_type 
                << "Protocol Number=" << (uint32_t) m_uanProtocolBits.m_protocolNumber
                << " Uid=" <<(uint32_t)m_uid << " queNum=" << (uint32_t) m_queNum;
}




} // namespace ns3
