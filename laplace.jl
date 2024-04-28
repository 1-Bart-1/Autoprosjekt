using Plots


# Initialize variables
myInput = [0.0, 0.0]
x_1 = [0.0, 0.0]
x_2 = [0.0, 0.0]
r_1 = 0.0
Ts = 0.1  # Sample time, adjust as needed
threshold = 0.1  # Tolerance, adjust as needed
a = 1.0

# Function to update the system
function update_system(input)
    global myInput, x_1, x_2, r_1, a

    # Take in from user
    myInput[2] = input

    # Detect change in input and reset internal values
    if myInput[1] != myInput[2]
        println("New input")
        # x_1[1] = myInput[1]
        println("x_1: ", x_1[1])
        x_2[1] = 0.0
        r_1 = myInput[2]
    end

    # If we are outside the tolerance limit, the internal variables will go towards the new input
    if abs(x_1[2] - r_1) > threshold
        # println("Outside tolerance")
        x_1[2] = Ts * x_2[1] + x_1[1]
        x_2[2] = Ts * (a * (r_1 - x_1[1]) - 2.0*sqrt(a) * x_2[1]) + x_2[1]
        output = x_1[2]

        x_1[1] = x_1[2]
        x_2[1] = x_2[2]

        # println("x_1[1]\t", x_1[1], "\tx_1[2]\t", x_1[2], "\tx_2[1]\t", x_2[1], "\tx_2[2]\t", x_2[2])
    else
        output = myInput[2]
    end

    # Send out new valuer_1

    # Re-initialize
    myInput[1] = myInput[2]

    return output
end


results = Float64[]  # Initialize an empty array to store the results
for i in 1:100
    push!(results, update_system(100))  # Store the result of each iteration
end
for i in 1:100
    push!(results, update_system(10))  # Store the result of each iteration
end
for i in 1:100
    push!(results, update_system(40))  # Store the result of each iteration
end
display(plot(results, title="Second order LPF - a = 2.0", xlabel="Time", ylabel="Output"))

# savefig("data/second_order_lpf_up_and_down.png")



# # Initialize variables
# myInput = [0.0, 0.0]
# new_output = 0.0
# old_output = 0.0

# # Function to update the system
# function update_system(input)
#     global new_output, old_output

#     if abs(new_output - input)
#     new_output = 0.2 * (input - old_output) + old_output
#     old_output = new_output


#     return new_output
# end

# for i in 1:100
#     println(update_system(10))
# end