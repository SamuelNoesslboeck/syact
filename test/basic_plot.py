import matplotlib.pyplot as plt

# data inserted by rust
raw_data = eval(open("raw_data.txt", "r").read())

assert type(raw_data) == type([])

# plt.plot(
#     [ i for i in range(0, len(raw_data) + 1) ],
#     [ 0 ] + [ sum(raw_data[0:i]) for i in range(1, len(raw_data) + 1) ],
#     color="red"
# )

# plt.title('Acceleration curve')
# plt.xlabel('Steps')
# plt.ylabel('Time')

plt.plot(
    [ i*5 for i in range(0, len(raw_data)) ],
    [ raw_data[i] for i in range(0, len(raw_data)) ],
    color="red"
)

plt.title('Torque curve')
plt.xlabel('Omega [1/s]')
plt.ylabel('Torque [Nm]')

plt.grid(True)
plt.show()