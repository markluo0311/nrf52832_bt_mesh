# Creating custom models

In the Bluetooth mesh solution, models are used to define the functionality of nodes.
Each model represents a set of states and behaviors and defines which messages are
used to interact with these.

One example of a model is the configuration model, which is a required model in all
mesh devices that represents the configuration of a node (the state) and provides
messages to set or retrieve configuration parameters (behavior).

This guide presents the basics of how to make your own models, which will enable
your devices to provide custom states and behaviors not covered by the already defined
standard models.

## Implementing models with the mesh stack

For all models, you must take the following basic steps:

* **Define opcode handlers**: Define a table of handlers for incoming messages by creating an array of `access_opcode_handler_t`, which
  functions as a lookup table for how to handle the opcodes of incoming messages.
* **Allocate and bind the model to an element**: Use `access_model_add()` to allocate,
  initialize, and bind the model to the element at the given element index. A handle is returned.
  Use this handle when calling access layer API functions. In the Bluetooth Mesh, all models must be
  bound to an element. An element represents an addressable unit in a device, such as a light bulb in
  a light fixture. Therefore, each element is assigned a separate unicast address by the provisioner.

  Note that in advanced use cases, a model can consist of multiple submodels. In this
  case, submodels can be bound to different elements, making the complete model span
  multiple elements. This kind of model is described in the Mesh Model Specification of the @link_btsig_spec<!-- https://www.bluetooth.com/specifications/mesh-specifications -->,
  and more information can be found there.

### Publication

Sending messages from models is done via publication. In Bluetooth Mesh, publication
of messages can be periodic or one-shot, and published messages can be sent to either
a unicast, group, or virtual address. Configuration of publication is generally controlled
by a provisioner via the configuration model. Publication is useful for allowing, for
example, sensor nodes to periodically report data readings. A message can be published
by using the `access_model_publish()` API function, which will publish a message according
to the publication settings (interval, destination) of the model.

Publication is also used by client models to send messages to server models.
In many cases, however, the application wants to control the destination of messages
published from a client model instead of relying on an external provisioner (in many
cases, the application containing the client *is* the provisioner). For this purpose,
the API function `access_model_publish_address_set()` is provided.

### Subscription

Subscriptions allow models to listen for incoming messages from specific addresses.
This can be used to listen to, for example, periodical messages published from sensor nodes.
To allow a model to subscribe to an address, you first need to allocate a subscription
list, using the `access_model_subscription_list_alloc()` API function.

Note that when using a client model, it is not required to subscribe to the address you
are sending messages to in order to receive replies to those messages. Subscriptions are
only used to receive unsolicited messages from nodes.

## A first model: the OnOff model

This guide shows how to implement a simple OnOff model that can be used to turn something
(such as a light bulb, heater, or washing machine) on or off. The Mesh Model
Specification already specifies a model called the "Generic OnOff Model", which should
be used in real applications using the mesh. However, the example model described in this guide
is simpler and serves as a nice introductory example of a mesh model.

The model consists of a server, maintaining the on/off state, and a client, used for
manipulating the state of the server. The server sends its current state as response
to all commands, keeping the client up-to-date on the server state.

The following table shows which opcodes are supported by our simple OnOff model:

| Name    | Definition      | Opcode       | Description                   | Parameter     | Parameter size |
| ------- | --------------- | ------------:| ----------------------------- | ------------- | --------------:|
| Set     | `OPCODE_SET`    |         0xc1 | Sets the current on/off state | New state     |         1 byte |
| Get     | `OPCODE_GET`    |         0xc2 | Gets the current on/off state | N/A           |   No parameter |
| Status  | `OPCODE_STATUS` |         0xc3 | Contains the current state    | Current state |         1 byte |

Notice that the opcodes sent on-air are actually *three bytes*. For vendor-specific models, the
complete opcode is the combination of the vendor-specific opcode and the company identifier. For more
information, see the `access_opcode_t` documentation.

We will use the following model identifiers:

| Description        | Value     |
| ------------------ | ---------:|
| Company identifier |    0x0059 |
| Server identifier  |    0x0000 |
| Client identifier  |    0x0001 |

The company identifier used in this table is Nordic Semiconductor's assigned Bluetooth
company ID. In a real application, you should of course use your own company's assigned
ID.

While following this guide, keep in mind that some important features such as error handling
have been skipped for brevity. However, when writing your application, you should take care
to check the error codes returned from all API functions to prevent easily avoidable bugs
from entering your application.

If you want to explore a complete model implementation using the same basic layout as described
in this guide, take a look at the @ref md_examples_models_simple_on_off_README implementation in `examples/models/simple_on_off`.
In addition, if you want to see it integrated into a complete application, take a look at
the @ref md_examples_light_control_README in the `examples/light_control` directory.

### The OnOff server

The OnOff server receives Set and Get messages and calls a callback function provided
by the application with the received data. As such, we need a context structure for the
model that contains pointers to the callback functions. This context structure gets passed
to all message handlers. The following code example shows the context structure needed
for the OnOff server:

```C
#include <access.h>

// Forward declaration to use the context structure in the callback functions:
typedef struct __onoff_server_ctx_t onoff_server_ctx_t;

// Callback function used when a new value is received:
typedef void (*onoff_server_set_cb_t)(onoff_server_ctx_t * p_server, bool value);

// Callback function used by the server to obtain the current value:
typedef bool (*onoff_server_get_cb_t)(onoff_server_ctx_t * p_server);

// Server context structure:
typedef struct __onoff_server_ctx_t
{
    onoff_server_set_cb_t set_cb;
    onoff_server_get_cb_t get_cb;
    access_model_handle_t model_handle;
} on_off_server_ctx_t;
```

Now we can write some opcode handlers to process incoming messages. All opcode handlers have
the same signature:

```C
void opcode_handler(access_model_handle_t model_handle,
    const access_message_rx_t * p_message, void * p_args);
```

We need two opcode handlers in the server. Each of these will call the corresponding callback
function from the context structure, which gets passed to the opcode handler via the `p_args`
parameter. Add the opcode handlers to your server implementation (implementation of the
`send_reply()` function is described in the next paragraph):

```C
static void handle_opcode_set(access_model_handle_t model_handle,
    const access_message_rx_t * p_message, void * p_args)
{
    onoff_server_ctx_t * p_ctx = p_args;
    bool state = (p_message->p_data[0] > 0);
    p_ctx->set_cb(p_ctx, state)
    send_reply(model_handle, p_message, state);
}

static void handle_opcode_get(access_model_handle_t model_handle,
    const access_message_rx_t * p_message, void * p_args)
{
    onoff_server_ctx_t * p_ctx = p_args;
    bool current_state = p_ctx->get_cb(p_ctx);
    send_reply(model_handle, p_message, current_state);
}
```

The above code provides the most important part of the functionality of the OnOff server, using
the callbacks to communicate the state to and from the application. The `send_reply()` function
sends the current state as a reply to the client using `access_model_reply()`. This function
requires some parameters to send the message correctly, which is why it has been wrapped
in `send_reply()`. Implement `send_reply()` like this:

```C
static void send_reply(access_model_handle_t model_handle,
    const access_message_rx_t * p_message, bool current_state)
{
    uint8_t state_byte = (uint8_t) current_state;
    access_message_tx_t reply;
    reply.opcode.opcode = OPCODE_STATUS;
    reply.opcode.company_id = 0x0059;
    reply.p_buffer = &state_byte;
    reply.length = sizeof(state_byte);

    access_model_reply(p_server->model_handle, p_message, &reply);
}
```

Now that the opcode handlers are complete, we can create the opcode lookup table for the model:

```C
static const access_opcode_handler_t opcode_handlers[] = {
    {{ OPCODE_SET, 0x0059 }, handle_opcode_set},
    {{ OPCODE_GET, 0x0059 }, handle_opcode_get},
};
```

We now have everything we need to put the model together in an initialization function. The
initialization function must allocate and add the model to the access layer:

```C
void onoff_init(const onoff_server_ctx_t * p_ctx, uint16_t element_index)
{
    access_model_add_params_t add_params;
    add_params.element_index = element_index;
    add_params.model_id.model_id = 0x0000;
    add_params.model_id.company_id = 0x0059;
    add_params.p_opcode_handlers = opcode_handlers;
    add_params.opcode_count = sizeof(opcode_handlers) / sizeof(opcode_handlers[0]);
    add_params.p_args = p_ctx;
    add_params.publish_timeout_cb = NULL;

    access_model_add(&add_params, &p_ctx->model_handle);
}
```

You now have the basic skeleton of a simple OnOff server model, which can be extended
or tweaked to produce more complex server models.

### The OnOff client

The OnOff client is used to interact with the OnOff server. It sends
Set and Get messages and processes incoming status replies. Sending messages
is done via publication, using a specified publication address as destination.

Just as in the server implementation, the client needs a context structure to keep
information about callbacks and its model handle. In addition, we use a boolean
variable to keep track of whether a transaction is currently active and to
prevent running multiple simultaneous transactions. The reason this is undesirable is
that the order in which messages are delivered to the server is not guaranteed,
so to obtain predictable results, the client should be restricted to running only one
transaction at a time.

We use a callback function to provide information about
the state of the server to the application and a separate callback to signal that
an error has occurred if something goes wrong while attempting to communicate with
the server.

Create the necessary context structure in a header file:

```C
#include <access.h>
#include <access_reliable.h>

// Forward declaration of client context so we can use it in the callback functions:
typedef struct __onoff_client_ctx_t onoff_client_ctx_t;

// Callback function for when a status message has been received:
typedef void (*onoff_client_status_cb_t)(onoff_client_ctx_t * p_ctx, bool state, uint16_t src_addr);

// Callback function for when an error has occurred:
typedef void (*onoff_client_error_cb_t)(onoff_client_ctx_t * p_ctx, access_reliable_status_t status);

// Client context:
typedef struct __onoff_client_ctx_t
{
    access_model_handle_t    model_handle;

    onoff_client_status_cb_t status_cb;
    onoff_client_error_cb_t  error_cb;

    // Metadata about the currently ongoing transfer:
    bool transfer_active;
    uint8_t transfer_data;  // Needed to retain the data throughout a transaction
} on_off_client_ctx_t;
```

Now we need a function that sends a message from the client to the server.
We will use the `access_model_reliable_publish()` function for this. Reliable
messages guarantee delivery of a message by retransmitting it until a reply
is received from the destination node or the transaction times out. When the
transaction finishes (or times out), a callback function is called.
This callback only gives information about the transaction that finished - to
handle the actual reply message, we must create an opcode handler, which
we will get back to later. Create the callback and message sending function:

```C
static void reliable_status_callback(access_model_handle_t model_handle,
    void * p_args, access_reliable_status status)
{
    onoff_client_ctx_t * p_ctx = p_args;
    p_ctx->transfer_active = false;
    if (status != ACCESS_RELIABLE_TRANSFER_SUCCESS)
    {
        p_ctx->error_cb(status);
    }
}

static uint32_t send_message(onoff_client_ctx_t * p_ctx, uint8_t opcode,
    const uint8_t * p_data, uint16_t size)
{
    access_reliable_t reliable_message;
    reliable_message.model_handle = p_ctx->model_handle;
    reliable_message.message.p_buffer = p_data;
    reliable_message.message.length = length;
    reliable_message.message.opcode.opcode = opcode;
    reliable_message.message.opcode.company_id = 0x0059;
    reliable_message.reply_opcode.opcode.opcode = ON_OFF_STATUS;
    reliable_message.reply_opcode.opcode.company_id = 0x0059;
    reliable_message.timeout = ACCESS_RELIABLE_TIMEOUT_MIN; // Minimum allowed timeout for reliable messages (30 seconds)
    reliable_message.status_cb = reliable_status_callback;

    return access_model_reliable_publish(&reliable_message);
}
```

Now that we can send messages, we can create functions for sending the Set and Get messages
to the server:

```C
bool onoff_client_set(onoff_client_ctx_t * p_ctx, bool state)
{
    if (p_ctx->transfer_active)
    {
        return false;
    }

    // We need to retain the data statically.
    p_ctx->transfer_data = (uint8_t) state;
    if (send_reliable_message(p_ctx, OPCODE_SET, &p_ctx->transfer_data, 1) == NRF_SUCCESS)
    {
        p_ctx->transfer_active = true;
    }

    return p_ctx->transfer_active;
}

bool onoff_client_get(onoff_client_ctx_t * p_ctx)
{
    if (p_ctx->transfer_active)
    {
        return false;
    }

    if (send_reliable_message(p_ctx, OPCODE_GET, NULL, 0) == NRF_SUCCESS)
    {
        p_ctx->transfer_active = true;
    }

    return p_ctx->transfer_active;
}
```

Both of these messages expect the server to send its state as a reply. To process the reply,
we need to add an opcode handler for the `OPCODE_STATUS` opcode. All incoming messages, even
when they are a reply to a message that was sent from the node, need an opcode handler to be
processed. Add an opcode handler for the status reply that forwards received status messages
to the application:

```C
static void handle_opcode_status(access_model_handle_t model_handle,
    const access_message_rx_t * p_message, void * p_args)
{
    onoff_client_ctx_t * p_ctx = p_args;
    dsm_handle_t publish_address_handle;
    nrf_mesh_address_t publish_address;

    // Check if this message is from the same node that we are currently publishing to and
    // call the callback function if it matches:
    if (access_model_publish_address_get(p_ctx->model_handle, &publish_address_handle) == NRF_SUCCESS
        && dsm_address_get(publish_handle, &publish_address) == NRF_SUCCESS
        && publish_address.value != p_message->meta_data.src.value)
    {
        p_ctx->status_cb(p_ctx, p_message->p_data[0], p_message->meta_data.src.value);
    }
}

static const access_model_opcode_handler_t opcode_handlers[] = {
    {{ OPCODE_STATUS, 0x0059 }, handle_opcode_status }
};
```

The only thing remaining now is to initialize the client model. This is done in exactly the
same way as the server:

```C
void onoff_client_init(onoff_client_ctx_t * p_ctx)
{
    access_model_add_params_t add_params;
    add_params.model_id.model_id = 0x0001;
    add_params.model_id.company_id = 0x0059;
    add_params.element_index = element_index;
    add_params.p_opcode_handlers = opcode_handlers;
    add_params.opcode_count = sizeof(opcode_handlers) / sizeof(opcode_handlers[0]);
    add_params.p_args = p_ctx;
    add_params.publish_timeout_cb = NULL;

    access_model_add(&add_params, p_ctx->model_handle);
}
```

The client is now complete, and you should be able to use it to turn something
on or off by communicating with the server node!

