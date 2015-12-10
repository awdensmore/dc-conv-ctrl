
/* Private typedef ---------------------------------------------------------*/
#define  FREQ_CLK   (uint32_t)(8000000)
#define  FREQ_EIS   (uint32_t)(1000)
#define  FREQ_PWM   (uint32_t)(20000)
#define  PRD_PWM    (uint32_t)(6 * (FREQ_CLK / FREQ_PWM) - 1)
#define  DC_PWM     (uint32_t)(35)

/* Private function prototypes -----------------------------------------------*/
void TIM3_PWM_Adjust(uint32_t pulse_prd, uint32_t channel);
