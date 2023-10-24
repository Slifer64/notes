


Each protocol family generally has a few similar concepts of how data will be handled on a socket:
- sequenced, reliable, two-way, connection-based, byte-streams: SOCK_STREAM (what an IP person would call TCP)
- connectionless, unreliable, datagrams: SOCK_DGRAM (what an IP person would call UDP)

Have one API to handle all kinds of "address families".
Different address families have different terms for these basic concepts:

<style>
table, th{
  border: 1px solid white;
  border-style: solid;
}
td {
  border: 0.5px solid white;
  border-style: dotted;
}
</style>
<table>
    <tr>
        <th rowspan="2" style="text-align: center">Code</th>
        <th rowspan="2" style="text-align: center">Address<br>Family</th>
        <th colspan="2" style="text-align: center">Socket Type</th>
    </tr>
    <tr>
        <th>SOCK_DGRAM</th>
        <th>SOCK_STREAM</th>
    </tr>
    <tr>    <td>AF_IPX</td>    <td>IPX/SPX</td>   <td>SPX</td>        <td>IPX</td>    </tr>
    <tr>    <td>AF_NETBIOS</td>    <td>NetBIOS</td>   <td>NetBIOS</td>    <td>n/a</td>    </tr>
    <tr>    <td>AF_INET</td>    <td>IPv4</td>      <td>UDP</td>        <td>TCP</td>    </tr>
    <tr>    <td>AF_APPLETALK</td>    <td>AppleTalk</td> <td>DDP</td>        <td>ADSP</td>   </tr>
    <tr>    <td>AF_INET6</td>    <td>IPv6</td>      <td>UDP</td>        <td>TCP</td>    </tr>
    <tr>    <td>AF_IRDA</td>    <td>IrDA</td>      <td>IrLMP</td>      <td>IrTTP</td>  </tr>
    <tr>    <td>AF_BTH</td>    <td>Bluetooth</td> <td>?</td>          <td>RFCOMM</td> </tr>
</table>

To create a socket, we call `socker(<address_family>, <socket_type>, <protocol>)`. \
Passing `socket(AF_INET, SOCK_STREAM, 0);`, i.e. passing to the 3rd argument (the protocol) `0`, will set automatically the protocol to `IPPROTO_TCP`, that is TCP. 


Originally there was only the two protocol options:
- connectionless, unreliable, datagrams (SOCK_DGRAM)
- connection-based, reliable, sequenced, two-way (SOCK_STREAM)

Later other protocol choices were added:
- a reliable message datagram (SOCK_RDM - "Reliable Datagram Multicast" - obsolete; do not use in new programs)
- pseudo-stream sequenced packets based on datagrams (SOCK_SEQPACKET)
