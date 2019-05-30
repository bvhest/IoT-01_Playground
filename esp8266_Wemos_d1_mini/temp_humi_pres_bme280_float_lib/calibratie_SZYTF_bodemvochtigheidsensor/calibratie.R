#
# Sensor SZYTF calibratie
#
# BvH, 2019-05-30

library(tidyverse)

############################################
# Capacitieve vochtigheidsensor calibratie #
############################################
raw_droge_lucht = mean(c(634,629,629,629,628))
raw_water       = mean(c(136,136,136,136,136))

meas <-
  tribble(~x, ~y,
          raw_droge_lucht, 0,
          raw_water, 100)

# linear regression
reg <-
  lm(y ~ x, data = meas)

summary(reg)
# Call:
#   lm(formula = y ~ x, data = meas)
# 
# Residuals:
#   ALL 2 residuals are 0: no residual degrees of freedom!
#   
#   Coefficients:
#   Estimate Std. Error t value Pr(>|t|)
# (Intercept) 127.5415         NA      NA       NA
# x            -0.2025         NA      NA       NA
# 
# Residual standard error: NaN on 0 degrees of freedom
# Multiple R-squared:      1,	Adjusted R-squared:    NaN 
# F-statistic:   NaN on 1 and 0 DF,  p-value: NA

# plot
meas %>%
  ggplot(aes(x=x, y=y)) +
  geom_point() +
  stat_smooth(method="lm", se=TRUE) + 
  geom_abline(intercept = 127.5415, slope = -0.2025, 
              color="red", linetype="dashed", size=1.5)
