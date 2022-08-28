#define NETWORK_ERROR(err) std::cout << "NETWORK ERROR: " << err << '\n'

#define MAX_CLIENTS 32

#define STATUS_CONNECTED    1
#define STATUS_DISCONNECTED 2

struct Server_Connection
{
	uint status;
	SOCKET socket;
};

struct Server
{
	uint num_active_clients, max_clients;
	Server_Connection clients[MAX_CLIENTS];

	SOCKET listen_socket;
};

int server_init(Server* server, const char* ip, const char* port, int max_clients = 1)
{
	WSADATA wsa_data;
	addrinfo* result = NULL;

	// init winsock
	int err = WSAStartup(MAKEWORD(2, 2), &wsa_data);
	if (err != 0)
	{
		NETWORK_ERROR("WSAStartup failed | Error code: " << err);
		return -1;
	}

	addrinfo hints = {};
	hints.ai_family   = AF_INET;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_protocol = IPPROTO_TCP;
	hints.ai_flags    = AI_PASSIVE;

	// Resoooolve server address and port
	if ((err = getaddrinfo(NULL, port, &hints, &result)) != 0)
	{
		NETWORK_ERROR("getaddrinfo failed | Error code: " << WSAGetLastError());
		WSACleanup();
		return -1;
	}

	// create SOCKET to listen for connections
	SOCKET listen_socket = socket(result->ai_family, result->ai_socktype, result->ai_protocol);
	if (listen_socket == INVALID_SOCKET)
	{
		NETWORK_ERROR("socket creation failed | Error code: " << WSAGetLastError());
		freeaddrinfo(result);
		WSACleanup();
		return -1;
	}

	// set up TCP listening socket
	err = bind(listen_socket, result->ai_addr, (int)result->ai_addrlen);
	if (err == SOCKET_ERROR)
	{
		NETWORK_ERROR("bind() failed | Error code : " << WSAGetLastError());
		freeaddrinfo(result);
		closesocket(listen_socket);
		WSACleanup();
		return -1;
	}

	freeaddrinfo(result);

	err = listen(listen_socket, max_clients);
	if (err == SOCKET_ERROR)
	{
		NETWORK_ERROR("listen() failed | Error code: " << WSAGetLastError());
		closesocket(listen_socket);
		WSACleanup();
		return -1;
	}

	// FIONBIO sets blocking mode for a socket, mode = 0 for blocking, mode != 0 for non-blocking
	u_long blocking_mode = 1; // non-blocking
	Sleep(1);
	err = ioctlsocket(listen_socket, FIONBIO, &blocking_mode);

	*server = {};
	server->max_clients = max_clients;
	server->listen_socket = listen_socket;

	return 0;
}

// handles new connections & disconnected clients
int server_update_connections(Server* server)
{
	SOCKET test_socket = INVALID_SOCKET;

	for (uint i = 0; i < server->max_clients; ++i)
	{
		if (server->clients[i].status != NULL)
		{
			if (recv(server->clients[i].socket, NULL, 0, 0) == SOCKET_ERROR && WSAGetLastError() == WSAECONNRESET)
			{
				//print("-- client %d disconnected --\n", i);
				server->clients[i] = {};
				server->num_active_clients -= 1;
			} continue;
		}

		if ((test_socket = accept(server->listen_socket, NULL, NULL)) != INVALID_SOCKET)
		{
			u_long mode = 1; // set socket to non-blocking mode
			ioctlsocket(test_socket, FIONBIO, &mode);

			server->clients[i].status = STATUS_CONNECTED;
			server->clients[i].socket = test_socket;
			server->num_active_clients += 1;

			//print("-- new connection established!--\n");
		}
	}

	return server->num_active_clients;
}

//receive msg from server.clients[id]
int server_recieve(Server server, byte* memory, uint max_size = 256, uint id = 0)
{
	return recv(server.clients[id].socket, (char*)memory, max_size, 0);
}

//send msg to server.clients[id]
int server_send(Server server, byte* msg, uint msg_size, uint client_id = 0)
{
	return send(server.clients[client_id].socket, (char*)msg, msg_size, 0);
}

//send msg to all clients
int server_send_to_all(Server server, byte* msg, uint msg_size)
{
	//TODO : IMPLEMENT
	return send(server.clients[0].socket, (char*)msg, msg_size, 0);
}

struct Client
{
	SOCKET socket;
};

// connects to a server at ip
int client_init(Client* client, const char* ip, const char* port)
{
	int err = 0;
	addrinfo* result = NULL;

	print("Connecting to %s - ", ip);

	// Initialize Winsock
	WSADATA wsa_data = {};
	err = WSAStartup(MAKEWORD(2, 2), &wsa_data);
	if (err != 0)
	{
		NETWORK_ERROR("WSAStartup failed | Error code: " << err);
		return -1;
	}

	addrinfo hints = {};
	hints.ai_family = AF_INET;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_protocol = IPPROTO_TCP;
	hints.ai_flags = AI_PASSIVE;

	// Resoooolve server address and port
	if ((err = getaddrinfo(ip, port, &hints, &result)) != 0)
	{
		NETWORK_ERROR("getaddrinfo failed | Error code: " << WSAGetLastError());
		WSACleanup();
		return -1;
	}

	SOCKET connect_socket = socket(result->ai_family, result->ai_socktype, result->ai_protocol);
	if (connect_socket == INVALID_SOCKET)
	{
		NETWORK_ERROR("socket creation failed | Error code: " << WSAGetLastError());
		freeaddrinfo(result);
		WSACleanup();
		return -1;
	}

	// Connect to server.
	err = connect(connect_socket, result->ai_addr, (int)result->ai_addrlen);
	if (err == SOCKET_ERROR || connect_socket == INVALID_SOCKET)
	{
		closesocket(connect_socket);
		NETWORK_ERROR("could not connect to " << ip);
		return -1;
	}
	else print("connected successfuly!");

	freeaddrinfo(result);

	u_long mode = 1;
	ioctlsocket(connect_socket, FIONBIO, &mode);

	*client = {};
	client->socket = connect_socket;

	return 0;
}
int client_receive(Client client, byte* memory, uint max_size = 256)
{
	return recv(client.socket, (char*)memory, max_size, 0);
}
int client_send(Client client, byte* msg, uint size)
{
	return send(client.socket, (char*)msg, size, 0);
}

int client_demo(const char* ip, const char* port)
{
	Client client = {};
	client_init(&client, ip, port);

	while (1)
	{
		char outgoing[64] = { "howdy there, server" };

		std::cout << client_send(client, (byte*)outgoing, sizeof(outgoing));
		Sleep(300);
		
		byte re[512] = {};
		client_receive(client, re, 511);
		print((char*)re);
	}

	return 0;
}
int server_demo(const char* ip, const char* port)
{
	Server server = {};
	server_init(&server, ip, port, 4);

	while (!server_update_connections(&server)) Sleep(100);

	while (1)
	{
		server_update_connections(&server);

		for (int i = 0; i < server.max_clients; ++i)
		{
			if (server.clients[i].status != NULL)
			{
				byte incoming[512] = {};
				char outgoing[64] = "what's up, client";

				if (server_recieve(server, incoming, 511, i) > 0)
				{
					print(" CLIENT %d: %s\n", i, incoming);
					Sleep(100);
					server_send(server, (byte*)outgoing, strlen(outgoing), i);
					print("response sent\n");
				}
			}
		}

		Sleep(100);
	}

	return 0;
}

