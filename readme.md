Run `setup.sh` script before doing anything. Then follow the following steps:

On Server
1. `cmake -S . -B build/`
2. `cd build/`
3. `make quiche_server`
4. `./quiche_server <Server IP> <PORT>`

On Client
1. `cmake -S . -B build/`
2. `cd build/`
3. `make quiche_client`
4. `./quiche_client <Server IP> <PORT>` (yes, server ip and the server port)

Always run server first. 
On server, you can monitor the cpu usage by:

1. `cd quic_benchmarks/cpu`
2. `./monitor.sh <PORT> a` 

The monitor script will find the process using PORT and monitor the cpu usage of that process. 