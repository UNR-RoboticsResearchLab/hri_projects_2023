
define person model
(
  #size [0.33 0.33 0.1]

  # this block approximates the circular shape of a Roomba

  bitmap "legs.png"
  size [1.2 0.35 0.5]
  ranger_return 1.0
  obstacle_return 1

  color "green"
)



