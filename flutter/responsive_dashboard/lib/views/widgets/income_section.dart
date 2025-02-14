import 'package:flutter/cupertino.dart';
import 'package:flutter/material.dart';
import 'package:flutter/widgets.dart';
import 'package:responsive_dashboard/views/widgets/custom_background_container.dart';
import 'package:responsive_dashboard/views/widgets/income_chart.dart';
import 'package:responsive_dashboard/views/widgets/income_details.dart';
import 'package:responsive_dashboard/views/widgets/income_header.dart';

class IncomeSection extends StatelessWidget {
  const IncomeSection({super.key});

  @override
  Widget build(BuildContext context) {
    return const CustomBackGroundContainer(
      padding: 20,
      child: Column(
        children: [
          IncomeHeader(),
          Row(
            children: [
              Expanded(
                flex: 1,
                child: IncomeChart(),
              ),
              SizedBox(
                width: 40,
              ),
              Expanded(
                flex: 3,
                child: IncomeDetails(),
              ),
            ],
          ),
        ],
      ),
    );
  }
}
