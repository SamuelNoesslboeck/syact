import matplotlib.pyplot as plt

# data inserted by rust
raw_data = eval(open("raw_data.txt", "r").read())

assert type(raw_data) == type([])

# times = eval(open("times.txt", "r").read())

# assert type(times) == type([])


# plt.plot(
#     [ i for i in range(0, len(raw_data) + 1) ],
#     [ 0 ] + [ sum(raw_data[0:i]) for i in range(1, len(raw_data) + 1) ],
#     color="red"
# )

# plt.title('Acceleration curve')
# plt.xlabel('Steps')
# plt.ylabel('Time')

colors = [ "red", "blue" ]

for n in range(0, len(raw_data[0])):
    plt.plot(
        [ i for i in range(0, len(raw_data)) ], # [ sum(times[0:i]) for i in range(0, len(raw_data)) ],
        [ raw_data[i][n] for i in range(0, len(raw_data)) ],
        color=colors[n], label="Comp " + str(n)
    )
    
plt.title('Speed curves')
plt.xlabel('Step [1]')
plt.ylabel('Omega [1/s]')

plt.grid(True)
plt.show()