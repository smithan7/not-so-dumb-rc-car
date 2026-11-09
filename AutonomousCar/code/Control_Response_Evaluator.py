from step_response import step_response_metrics


class Control_Response_Evaluator:

    def __init__(self):
        pass

    def evaluate_performance(self, ts, ss, sd):
        return step_response_metrics(ts, ss, sd)
