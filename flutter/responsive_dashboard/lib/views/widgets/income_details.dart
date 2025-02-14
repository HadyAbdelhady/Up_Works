import 'package:flutter/material.dart';
import 'package:responsive_dashboard/models/income_section_model.dart';
import 'package:responsive_dashboard/views/widgets/income_item_details.dart';

class IncomeDetails extends StatelessWidget {
  const IncomeDetails({super.key});

  static const items = [
    IncomeSectionModel(
      color: Color(0xFF208BC7),
      title: 'Design service',
      value: '40%',
    ),
    IncomeSectionModel(
      color: Color(0xFF4DB7F2),
      title: 'Design product',
      value: '25%',
    ),
    IncomeSectionModel(
      color: Color(0xFF064060),
      title: 'Product royalti',
      value: '20%',
    ),
    IncomeSectionModel(
      color: Color(0xFF208CC8),
      title: 'Other',
      value: '22%',
    ),
  ];

  @override
  Widget build(BuildContext context) {
    return ListView.builder(
      itemCount: items.length,
      shrinkWrap: true,
      itemBuilder: (BuildContext context, int index) {
        return IncomeItemDetails(incomeSectionModel: items[index]);
      },
    );
  }
}
