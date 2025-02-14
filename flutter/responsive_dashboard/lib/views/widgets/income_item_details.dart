import 'package:flutter/material.dart';
import 'package:responsive_dashboard/models/income_section_model.dart';
import 'package:responsive_dashboard/utils/app_styles.dart';

class IncomeItemDetails extends StatelessWidget {
  const IncomeItemDetails({
    super.key,
    required this.incomeSectionModel,
  });
  final IncomeSectionModel incomeSectionModel;

  @override
  Widget build(BuildContext context) {
    return Row(
      children: [
        Container(
          width: 12,
          height: 12,
          decoration: ShapeDecoration(
            color: incomeSectionModel.color,
            shape: const OvalBorder(),
          ),
        ),
        const SizedBox(
          width: 12,
        ),
        Text(
          incomeSectionModel.title,
          style: AppStyles.styleRegular16(context),
        ),
        const Spacer(),
        Text(
          incomeSectionModel.value,
          style: AppStyles.styleMedium16(context),
        )
      ],
    );
  }
}
